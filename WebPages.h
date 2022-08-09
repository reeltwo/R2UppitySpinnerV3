#ifdef USE_WIFI_WEB
#include "web-images.h"

bool sOTAInProgress;
int sLifterSpeed = 50;
int sLifterHeight = 0;
int sRotationSpeed = 50;
int sCurrentSeq = 0;
int sRotatePeriscope = 0;

WMenuData mainMenu[] = {
    { "Periscope", "/periscope" },
    { "Setup", "/setup" }
};

WMenuData setupMenu[] = {
    { "Home", "/" },
    { "Calibrate", "/calibrate" },
    { "Marcduino", "/marcduino" },
    { "WiFi", "/wifi" },
    { "Firmware", "/firmware" },
    { "Back", "/" }
};

WElement mainContents[] = {
    WVerticalMenu("menu", mainMenu, SizeOfArray(mainMenu)),
    rseriesSVG
};

WElement setupContents[] = {
    WVerticalMenu("setup", setupMenu, SizeOfArray(setupMenu)),
    rseriesSVG
};

String sStoredSeq[] = {
    "None",
    "Sequence #0",
    "Sequence #1",
    "Sequence #2",
    "Sequence #3",
    "Sequence #4",
    "Sequence #5",
    "Sequence #6",
    "Sequence #7",
    "Sequence #8"
};

String sLightKitSeq[] = {
    "Full Cycle",
    "Off",
    "Obi Wan",
    "Yoda",
    "Sith",
    "Search Light",
    "Dagobah",
    "Sparkle"
};

WElement periscopeContents[] = {
    WSelect("Light Kit Sequence", "lightkit",
        sLightKitSeq, SizeOfArray(sLightKitSeq),
        []() { return lifter.getLightShow(); },
        [](int val) {
            lifter.setLightShow(val);
        } ),
    WSelect("Periscope Sequence", "sequence",
        sStoredSeq, SizeOfArray(sStoredSeq),
        []() { return sCurrentSeq; },
        [](int val) {
            sCurrentSeq = val;
            if (val != 0)
            {
                executeCommand(":PS%d", val-1);
            }
        } ),
    WSlider("Lifter Speed", "lifterspeed", 0, 100,
        []()->int { return sLifterSpeed; },
        [](int val) { sLifterSpeed = val; } ),
    WSlider("Rotation Speed", "rotatespeed", 0, 100,
        []()->int { return sRotationSpeed; },
        [](int val) { sRotationSpeed = val; } ),
    WSlider("Lifter Height", "lifterheight", 0, 100,
        []()->int { return sLifterHeight; },
        [](int val) {
            sLifterHeight = val;
            executeCommand(":PP%d,%d", val, sLifterSpeed);
        } ),
    WSlider("Rotate Periscope", "rotate", 0, 359,
        []()->int { return sRotatePeriscope; },
        [](int val) {
            sRotatePeriscope = val;
            executeCommand(":PA%d,%d", val, sRotationSpeed);
        } ),

    WButton("Spin Left", "spinleft", []() {
        executeCommand(":PR%d", sRotationSpeed);
    }),
    WHorizontalAlign(),
    WButton("Spin Right", "spinright", []() {
        executeCommand(":PR%d", -sRotationSpeed);
    }),
    WHorizontalAlign(),
    WButton("Random", "random", []() {
        executeCommand(":PM%d,%d,2,4", sLifterSpeed, sRotationSpeed);
    }),
    WHorizontalAlign(),
    WButton("Down", "down", []() {
        executeCommand(":PH");
    }),
    WVerticalAlign(),

    WButton("Back", "back", "/"),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    rseriesSVG
};

WElement calibrateContents[] = {
    WButton("Calibrate", "calibrate", []() {
        executeCommand("#PSC");
    }),
    WButton("Back", "back", "/setup"),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    WVerticalAlign(),
    rseriesSVG
};

String swBaudRates[] = {
    "2400",
    "9600",
};

int marcSerialBaud;
bool marcWifiEnabled;

WElement marcduinoContents[] = {
    WSelect("Serial Baud Rate", "serialbaud",
        swBaudRates, SizeOfArray(swBaudRates),
        []() { return (marcSerialBaud = (preferences.getInt(PREFERENCE_MARCSERIAL, MARC_SERIAL_BAUD_RATE)) == 2400) ? 0 : 1; },
        [](int val) { marcSerialBaud = (val == 0) ? 2400 : 9600; } ),
    WCheckbox("Marcduino on Wifi (port 2000)", "wifienabled",
        []() { return (marcWifiEnabled = (preferences.getBool(PREFERENCE_MARCWIFI_ENABLED, MARC_WIFI_ENABLED))); },
        [](bool val) { marcWifiEnabled = val; } ),
    WButton("Save", "save", []() {
        preferences.putInt(PREFERENCE_MARCSERIAL, marcSerialBaud);
        preferences.putBool(PREFERENCE_MARCWIFI_ENABLED, marcWifiEnabled);
    }),
    WHorizontalAlign(),
    WButton("Back", "back", "/setup"),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    WVerticalAlign(),
    rseriesSVG
};

String wifiSSID;
String wifiPass;
bool wifiAP;
bool wifiEnabled;

WElement wifiContents[] = {
    W1("WiFi Setup"),
    WCheckbox("WiFi Enabled", "enabled",
        []() { return (wifiEnabled = preferences.getBool(PREFERENCE_WIFI_ENABLED, WIFI_ENABLED)); },
        [](bool val) { wifiEnabled = val; } ),
    WCheckbox("Access Point", "apmode",
        []() { return (wifiAP = preferences.getBool(PREFERENCE_WIFI_AP, WIFI_ACCESS_POINT)); },
        [](bool val) { wifiAP = val; } ),
    WTextField("WiFi:", "wifi",
        []()->String { return (wifiSSID = preferences.getString(PREFERENCE_WIFI_SSID, WIFI_AP_NAME)); },
        [](String val) { wifiSSID = val; } ),
    WPassword("Password:", "password",
        []()->String { return (wifiPass = preferences.getString(PREFERENCE_WIFI_PASS, WIFI_AP_PASSPHRASE)); },
        [](String val) { wifiPass = val; } ),
    WButton("Save", "save", []() {
        DEBUG_PRINTLN("WiFi Changed");
        preferences.putBool(PREFERENCE_WIFI_ENABLED, wifiEnabled);
        preferences.putBool(PREFERENCE_WIFI_AP, wifiAP);
        preferences.putString(PREFERENCE_WIFI_SSID, wifiSSID);
        preferences.putString(PREFERENCE_WIFI_PASS, wifiPass);
        DEBUG_PRINTLN("Restarting");
        preferences.end();
        ESP.restart();
    }),
    WHorizontalAlign(),
    WButton("Back", "back", "/setup"),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    WVerticalAlign(),
    rseriesSVG
};

WElement firmwareContents[] = {
    W1("Firmware Setup"),
    WFirmwareFile("Firmware:", "firmware"),
    WFirmwareUpload("Reflash", "firmware"),
    WLabel("Current Firmware Build Date:", "label"),
    WLabel(__DATE__, "date"),
#ifdef BUILD_VERSION
    WHRef(BUILD_VERSION, "Sources"),
#endif
    WButton("Clear Prefs", "clear", []() {
        DEBUG_PRINTLN("Clear all preference settings");
        preferences.clear();
    }),
    WHorizontalAlign(),
    WButton("Reboot", "reboot", []() {
        DEBUG_PRINTLN("Rebooting");
        preferences.end();
        ESP.restart();
    }),
    WHorizontalAlign(),
    WButton("Back", "back", "/setup"),
    WHorizontalAlign(),
    WButton("Home", "home", "/"),
    WVerticalAlign(),
    rseriesSVG
};

WPage pages[] = {
    WPage("/", mainContents, SizeOfArray(mainContents)),
      WPage("/periscope", periscopeContents, SizeOfArray(periscopeContents)),
    WPage("/setup", setupContents, SizeOfArray(setupContents)),
      WPage("/calibrate", calibrateContents, SizeOfArray(calibrateContents)),
      WPage("/marcduino", marcduinoContents, SizeOfArray(marcduinoContents)),
      WPage("/wifi", wifiContents, SizeOfArray(wifiContents)),
      WPage("/firmware", firmwareContents, SizeOfArray(firmwareContents)),
        WUpload("/upload/firmware",
            [](Client& client)
            {
                if (Update.hasError())
                    client.println("HTTP/1.0 200 FAIL");
                else
                    client.println("HTTP/1.0 200 OK");
                client.println("Content-type:text/html");
                client.println("Vary: Accept-Encoding");
                client.println();
                client.println();
                client.stop();
                if (!Update.hasError())
                {
                    delay(1000);
                    preferences.end();
                    ESP.restart();
                }
                sOTAInProgress = false;
            },
            [](WUploader& upload)
            {
                if (upload.status == UPLOAD_FILE_START)
                {
                    sOTAInProgress = true;
                    unmountFileSystems();
                    Serial.printf("Update: %s\n", upload.filename.c_str());
                    if (!Update.begin(upload.fileSize))
                    {
                        //start with max available size
                        Update.printError(Serial);
                    }
                }
                else if (upload.status == UPLOAD_FILE_WRITE)
                {
                #ifdef USE_DEBUG
                    float range = (float)upload.receivedSize / (float)upload.fileSize;
                    DEBUG_PRINTLN("Received: "+String(range*100)+"%");
                #endif
                    /* flashing firmware to ESP*/
                    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
                    {
                        Update.printError(Serial);
                    }
                }
                else if (upload.status == UPLOAD_FILE_END)
                {
                    DEBUG_PRINTLN("GAME OVER");
                    if (Update.end(true))
                    {
                        //true to set the size to the current progress
                        Serial.printf("Update Success: %u\nRebooting...\n", upload.receivedSize);
                    }
                    else
                    {
                        Update.printError(Serial);
                    }
                }
            })
};
WifiWebServer<10,SizeOfArray(pages)> webServer(pages, wifiAccess);
#endif
