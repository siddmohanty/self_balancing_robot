// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT license.

import * as fs from "fs";
import * as path from "path";
import * as url from "url";
import * as vscode from "vscode";
import * as util from "../common/util";

import * as constants from "../common/constants";
import { arduinoChannel } from "../common/outputChannel";
import { DeviceContext } from "../deviceContext";
import { ArduinoApp } from "./arduino";
import { IArduinoSettings } from "./arduinoSettings";
import { parseBoardDescriptor } from "./board";
import { IBoard, IPackage, IPlatform } from "./package";
import { VscodeSettings } from "./vscodeSettings";

export class BoardManager {

    private _packages: IPackage[];

    private _platforms: IPlatform[];

    private _installedPlatforms: IPlatform[];

    private _boards: Map<string, IBoard>;

    private _boardConfigStatusBar: vscode.StatusBarItem;

    private _currentBoard: IBoard;

    private _onBoardTypeChanged = new vscode.EventEmitter<void>();

    constructor(private _settings: IArduinoSettings, private _arduinoApp: ArduinoApp) {
        this._boardConfigStatusBar = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Right, constants.statusBarPriority.BOARD);
        this._boardConfigStatusBar.command = "arduino.showBoardConfig";
        this._boardConfigStatusBar.tooltip = "Show Board Config";
    }

    public async loadPackages(update: boolean = false) {
        this._packages = [];
        this._platforms = [];
        this._installedPlatforms = [];

        const additionalUrls = this.getAdditionalUrls();
        if (update) { // Update index files.
            await this.setPreferenceUrls(additionalUrls);
            await this._arduinoApp.initialize(true);
        }

        // Parse package index files.
        const indexFiles = ["package_index.json"].concat(additionalUrls);
        const rootPackageFolder = this._settings.packagePath;
        for (const indexFile of indexFiles) {
            const indexFileName = this.getIndexFileName(indexFile);
            if (!indexFileName) {
                continue;
            }
            if (!update && !util.fileExistsSync(path.join(rootPackageFolder, indexFileName))) {
                await this.setPreferenceUrls(additionalUrls);
                await this._arduinoApp.initialize(true);
            }
            this.loadPackageContent(indexFileName);
        }

        // Load default platforms from arduino installation directory and user manually installed platforms.
        this.loadInstalledPlatforms();

        // Load all supported boards type.
        this.loadInstalledBoards();
        this.updateStatusBar();
        this._boardConfigStatusBar.show();

        const dc = DeviceContext.getInstance();
        dc.onDidChange(() => {
            this.updateStatusBar();
        });
    }

    public async changeBoardType() {
        const supportedBoardTypes = this.listBoards();
        if (supportedBoardTypes.length === 0) {
            vscode.window.showInformationMessage("No supported board is available.");
            return;
        }
        // TODO:? Add separator item between different platforms.
        const chosen = await vscode.window.showQuickPick(<vscode.QuickPickItem[]>supportedBoardTypes.map((entry): vscode.QuickPickItem => {
            return <vscode.QuickPickItem>{
                label: entry.name,
                description: entry.platform.name,
                entry,
            };
        }).sort((a, b): number => {
            if (a.description === b.description) {
                return a.label === b.label ? 0 : (a.label > b.label ? 1 : -1);
            } else {
                return a.description > b.description ? 1 : -1;
            }
        }), { placeHolder: "Select board type" });
        if (chosen && chosen.label) {
            this.doChangeBoardType((<any>chosen).entry);
        }
    }

    public async updatePackageIndex(indexUri: string): Promise<boolean> {
        let allUrls = this.getAdditionalUrls();
        if (!(allUrls.indexOf(indexUri) >= 0)) {
            allUrls = allUrls.concat(indexUri);
            await VscodeSettings.getInstance().updateAdditionalUrls(allUrls);
            await this._arduinoApp.setPref("boardsmanager.additional.urls", this.getAdditionalUrls().join(","));
        }
        return true;
    }

    public get onBoardTypeChanged(): vscode.Event<void> {
        return this._onBoardTypeChanged.event;
    }

    public doChangeBoardType(targetBoard: IBoard) {
        const dc = DeviceContext.getInstance();
        dc.board = targetBoard.key;
        this._currentBoard = targetBoard;
        dc.configuration = this._currentBoard.customConfig;
        this._boardConfigStatusBar.text = targetBoard.name;
        this._arduinoApp.addLibPath(null);

        this._onBoardTypeChanged.fire();
    }

    public get packages(): IPackage[] {
        return this._packages;
    }

    public get platforms(): IPlatform[] {
        return this._platforms;
    }

    public get installedBoards(): Map<string, IBoard> {
        return this._boards;
    }

    public get currentBoard(): IBoard {
        return this._currentBoard;
    }

    public getInstalledPlatforms(): any[] {
        // Always using manually installed platforms to overwrite the same platform from arduino installation directory.
        const installedPlatforms = this.getDefaultPlatforms();

        const mergePlatform = (plat) => {
            const find = installedPlatforms.find((_plat) => {
                return _plat.packageName === plat.packageName && _plat.architecture === plat.architecture;
            });
            if (!find) {
                installedPlatforms.push(plat);
            } else {
                find.defaultPlatform = plat.defaultPlatform;
                find.version = plat.version;
                find.rootBoardPath = plat.rootBoardPath;
            }
        };

        const customPlatforms = this.getCustomPlatforms();
        const manuallyInstalled = this.getManuallyInstalledPlatforms();

        customPlatforms.forEach(mergePlatform);
        manuallyInstalled.forEach(mergePlatform);

        return installedPlatforms;
    }

    public loadPackageContent(indexFile: string): void {
        const indexFileName = this.getIndexFileName(indexFile);
        if (!util.fileExistsSync(path.join(this._settings.packagePath, indexFileName))) {
            return;
        }
        const packageContent = fs.readFileSync(path.join(this._settings.packagePath, indexFileName), "utf8");
        if (!packageContent) {
            return;
        }

        let rawModel = null;
        try {
            rawModel = JSON.parse(packageContent);
        } catch (ex) {
            arduinoChannel.error(`Invalid json file "${path.join(this._settings.packagePath, indexFileName)}".
            Suggest to remove it manually and allow boardmanager to re-download it.`);
            return;
        }

        if (!rawModel || !rawModel.packages || !rawModel.packages.length) {
            return;
        }

        this._packages = this._packages.concat(rawModel.packages);

        rawModel.packages.forEach((pkg) => {
            pkg.platforms.forEach((plat) => {
                plat.package = pkg;
                const addedPlatform = this._platforms
                    .find((_plat) => _plat.architecture === plat.architecture && _plat.package.name === plat.package.name);
                if (addedPlatform) {
                    // union boards from all versions.
                    // We should not union boards: https://github.com/Microsoft/vscode-arduino/issues/414
                    // addedPlatform.boards = util.union(addedPlatform.boards, plat.boards, (a, b) => {
                    //     return a.name === b.name;
                    // });
                    if (addedPlatform.name === plat.name) {
                        addedPlatform.versions.push(plat.version);
                    }
                } else {
                    plat.versions = [plat.version];
                    // Clear the version information since the plat will be used to contain all supported versions.
                    plat.version = "";
                    this._platforms.push(plat);
                }
            });
        });
    }

    public updateInstalledPlatforms(pkgName: string, arch: string) {
        const archPath = path.join(this._settings.packagePath, "packages", pkgName, "hardware", arch);

        const allVersion = util.filterJunk(util.readdirSync(archPath, true));
        if (allVersion && allVersion.length) {
            const newPlatform = {
                packageName: pkgName,
                architecture: arch,
                version: allVersion[0],
                rootBoardPath: path.join(archPath, allVersion[0]),
                defaultPlatform: false,
            };

            const existingPlatform = this._platforms.find((_plat) => {
                return _plat.package.name === pkgName && _plat.architecture === arch;
            });
            if (existingPlatform) {
                existingPlatform.defaultPlatform = newPlatform.defaultPlatform;
                if (!existingPlatform.installedVersion) {
                    existingPlatform.installedVersion = newPlatform.version;
                    existingPlatform.rootBoardPath = newPlatform.rootBoardPath;
                    this._installedPlatforms.push(existingPlatform);
                }
                this.loadInstalledBoardsFromPlatform(existingPlatform);
            }
        }
    }

    public updateStatusBar(show: boolean = true): void {
        if (show) {
            this._boardConfigStatusBar.show();
            const dc = DeviceContext.getInstance();
            const selectedBoard = this._boards.get(dc.board);
            if (selectedBoard) {
                this._currentBoard = selectedBoard;
                this._boardConfigStatusBar.text = selectedBoard.name;
                if (dc.configuration) {
                    this._currentBoard.loadConfig(dc.configuration);
                }
            } else {
                this._currentBoard = null;
                this._boardConfigStatusBar.text = "<Select Board Type>";
            }
        } else {
            this._boardConfigStatusBar.hide();
        }
    }

    private loadInstalledPlatforms() {
        const installed = this.getInstalledPlatforms();
        installed.forEach((platform) => {
            const existingPlatform = this._platforms.find((_plat) => {
                return _plat.package.name === platform.packageName && _plat.architecture === platform.architecture;
            });
            if (existingPlatform) {
                existingPlatform.defaultPlatform = platform.defaultPlatform;
                if (!existingPlatform.installedVersion) {
                    existingPlatform.installedVersion = platform.version;
                    existingPlatform.rootBoardPath = platform.rootBoardPath;
                    this._installedPlatforms.push(existingPlatform);
                }
            } else {
                platform.installedVersion = platform.version;
                this._installedPlatforms.push(platform);
            }
        });
    }

    // Default arduino package information from arduino installation directory.
    private getDefaultPlatforms(): IPlatform[] {
        const defaultPlatforms = [];
        try {
            const packageBundled = fs.readFileSync(path.join(this._settings.defaultPackagePath, "package_index_bundled.json"), "utf8");
            if (!packageBundled) {
                return defaultPlatforms;
            }
            const bundledObject = JSON.parse(packageBundled);
            if (bundledObject && bundledObject.packages) {
                for (const pkg of bundledObject.packages) {
                    for (const platform of pkg.platforms) {
                        if (platform.version) {
                            defaultPlatforms.push({
                                packageName: pkg.name,
                                architecture: platform.architecture,
                                version: platform.version,
                                rootBoardPath: path.join(this._settings.defaultPackagePath, pkg.name, platform.architecture),
                                defaultPlatform: true,
                            });
                        }
                    }
                }
            }
        } catch (ex) {
        }
        return defaultPlatforms;
    }

    private getCustomPlatforms(): IPlatform[] {
        const customPlatforms = [];
        const hardwareFolder = path.join(this._settings.sketchbookPath, "hardware");
        if (!util.directoryExistsSync(hardwareFolder)) {
            return customPlatforms;
        }

        const dirs = util.filterJunk(util.readdirSync(hardwareFolder, true)); // in Mac, filter .DS_Store file.
        if (!dirs || dirs.length < 1) {
            return customPlatforms;
        }
        for (const packageName of dirs) {
            const architectures = util.filterJunk(util.readdirSync(path.join(hardwareFolder, packageName), true));
            if (!architectures || architectures.length < 1) {
                continue;
            }
            architectures.forEach((architecture) => {
                const platformFolder = path.join(hardwareFolder, packageName, architecture);
                if (util.fileExistsSync(path.join(platformFolder, "boards.txt")) && util.fileExistsSync(path.join(platformFolder, "platform.txt"))) {
                    const configs = util.parseConfigFile(path.join(platformFolder, "platform.txt"));
                    customPlatforms.push({
                        packageName,
                        architecture,
                        version: configs.get("version"),
                        rootBoardPath: path.join(hardwareFolder, packageName, architecture),
                        defaultPlatform: false,
                    });
                }
            });
        }
        return customPlatforms;
    }

    // User manually installed packages.
    private getManuallyInstalledPlatforms(): any[] {
        const manuallyInstalled = [];
        const rootPackagePath = path.join(path.join(this._settings.packagePath, "packages"));
        if (!util.directoryExistsSync(rootPackagePath)) {
            return manuallyInstalled;
        }
        const dirs = util.filterJunk(util.readdirSync(rootPackagePath, true)); // in Mac, filter .DS_Store file.
        for (const packageName of dirs) {
            const archPath = path.join(this._settings.packagePath, "packages", packageName, "hardware");
            if (!util.directoryExistsSync(archPath)) {
                continue;
            }
            const architectures = util.filterJunk(util.readdirSync(archPath, true));
            architectures.forEach((architecture) => {
                const allVersion = util.filterJunk(util.readdirSync(path.join(archPath, architecture), true));
                if (allVersion && allVersion.length) {
                    manuallyInstalled.push({
                        packageName,
                        architecture,
                        version: allVersion[0],
                        rootBoardPath: path.join(archPath, architecture, allVersion[0]),
                        defaultPlatform: false,
                    });
                }
            });
        }
        return manuallyInstalled;
    }

    private loadInstalledBoards(): void {
        this._boards = new Map<string, IBoard>();
        this._installedPlatforms.forEach((plat) => {
            this.loadInstalledBoardsFromPlatform(plat);
        });
    }

    private loadInstalledBoardsFromPlatform(plat: IPlatform) {
        if (util.fileExistsSync(path.join(plat.rootBoardPath, "boards.txt"))) {
            const boardContent = fs.readFileSync(path.join(plat.rootBoardPath, "boards.txt"), "utf8");
            const res = parseBoardDescriptor(boardContent, plat);
            res.forEach((bd) => {
                this._boards.set(bd.key, bd);
            });
        }
    }

    private listBoards(): IBoard[] {
        const result = [];
        this._boards.forEach((b) => {
            result.push(b);
        });
        return result;
    }

    private getIndexFileName(uriString: string): string {
        if (!uriString) {
            return;
        }
        const normalizedUrl = url.parse(uriString);
        if (!normalizedUrl) {
            return;
        }
        return normalizedUrl.pathname.substr(normalizedUrl.pathname.lastIndexOf("/") + 1);
    }

    private getAdditionalUrls(): string[] {
        function formatUrls(urls): string[] {
            if (urls) {
                let _urls: string[];

                if (!Array.isArray(urls) && typeof urls === "string") {
                    _urls = (<string>urls).split(",");
                } else {
                    _urls = <string[]>urls;
                }

                return util.trim(_urls);
            }
            return [];
        }
        // For better compatibility, merge urls both in user settings and arduino IDE preferences.
        const settingsUrls = formatUrls(VscodeSettings.getInstance().additionalUrls);
        let preferencesUrls = [];
        const preferences = this._settings.preferences;
        if (preferences && preferences.has("boardsmanager.additional.urls")) {
            preferencesUrls = formatUrls(preferences.get("boardsmanager.additional.urls"));
        }
        return util.union(settingsUrls, preferencesUrls);
    }

    private async setPreferenceUrls(additionalUrls: string[]) {
        const settingsUrls = additionalUrls.join(",");
        if (this._settings.preferences.get("boardsmanager.additional.urls") !== settingsUrls) {
            await this._arduinoApp.setPref("boardsmanager.additional.urls", settingsUrls);
        }
    }
}
