// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT license.

import * as fs from "fs";
import * as path from "path";
import * as vscode from "vscode";
import * as constants from "../common/constants";
import * as util from "../common/util";

import { VscodeSettings } from "../arduino/vscodeSettings";
import ArduinoActivator from "../arduinoActivator";
import ArduinoContext from "../arduinoContext";
import { ArduinoWorkspace } from "../common/workspace";

export class CompletionProvider implements vscode.CompletionItemProvider {

    private _headerFiles = new Set<string>();

    private _libPaths = new Set<string>();

    private _watcher: vscode.FileSystemWatcher;

    private _cppConfigFile: string;

    private _activated: boolean = false;

    constructor() {
        if (vscode.workspace && ArduinoWorkspace.rootPath) {
            this._cppConfigFile = path.join(ArduinoWorkspace.rootPath, constants.CPP_CONFIG_FILE);
            this._watcher = vscode.workspace.createFileSystemWatcher(this._cppConfigFile);
            this._watcher.onDidCreate(() => this.updateLibList());
            this._watcher.onDidChange(() => this.updateLibList());
            this._watcher.onDidDelete(() => this.updateLibList());
        }
    }

    public async provideCompletionItems(document: vscode.TextDocument, position: vscode.Position):
         Promise<vscode.CompletionItem[]> {
        if (VscodeSettings.getInstance().skipHeaderProvider) {
            return [];
        }
        if (!ArduinoContext.initialized) {
            await ArduinoActivator.activate();
        }
        if (!this._activated) {
            this._activated = true;
            this.updateLibList();
        }
        // Check if we are currently inside an include statement.
        const text = document.lineAt(position.line).text.substr(0, position.character);
        const match = text.match(/^\s*#\s*include\s*(<[^>]*|"[^"]*)$/);

        if (match) {
            const result = [];
            this._headerFiles.forEach((headerFile) => {
                result.push(new vscode.CompletionItem(headerFile, vscode.CompletionItemKind.File));
            });
            return result;
        }
    }

    private updateLibList(): void {
        if (!this._activated) {
            return;
        }
        this._libPaths.clear();
        this._headerFiles.clear();
        ArduinoContext.arduinoApp.getDefaultPackageLibPaths().forEach((defaultPath) => {
            this._libPaths.add(defaultPath);
        });

        if (fs.existsSync(this._cppConfigFile)) {
            const deviceConfig = util.tryParseJSON(fs.readFileSync(this._cppConfigFile, "utf8"));
            if (deviceConfig) {
                if (deviceConfig.sketch) {
                    const appFolder = path.dirname(deviceConfig.sketch);
                    if (util.directoryExistsSync(appFolder)) {
                        this._libPaths.add(path.normalize(appFolder));
                    }
                }
                if (deviceConfig.configurations) {
                    const plat = util.getCppConfigPlatform();
                    deviceConfig.configurations.forEach((configSection) => {
                        if (configSection.name === plat && Array.isArray(configSection.includePath)) {
                            configSection.includePath.forEach((includePath) => {
                                this._libPaths.add(path.normalize(includePath));
                            });
                        }
                    });
                }
            }
        }

        this._libPaths.forEach((includePath) => {
            this.addLibFiles(includePath);
        });
    }

    private addLibFiles(libPath: string): void {
        if (!util.directoryExistsSync(libPath)) {
            return;
        }
        const subItems = fs.readdirSync(libPath);
        subItems.forEach((item) => {
            try {
                const state = fs.statSync(path.join(libPath, item));
                if (state.isFile() && item.endsWith(".h")) {
                    this._headerFiles.add(item);
                } else if (state.isDirectory()) {
                    this.addLibFiles(path.join(libPath, item));
                }
            } catch (ex) {
            }
        });
    }
}
