///////////////////////////////////////////////////////////////////////////////
//
// Settings menu selection
// 
///////////////////////////////////////////////////////////////////////////////

static const char* sSettingsMenu[] = {
#ifdef USE_WIFI
    "Set WiFi\nMode",
#endif
    "Calibrate\nLifter",
    "Erase All\nSettings"
};

class SettingsScreen : public MenuScreen
{
public:
    enum {
    #ifdef USE_WIFI
        kSetWifiMode,
    #endif
        kCalibrateLifter,
        kEraseAllSettings
    };
    SettingsScreen() :
        MenuScreen(kSettingsScreen, sSettingsMenu, SizeOfArray(sSettingsMenu))
    {}

    virtual void buttonInReleased() override
    {
        switch (fCurrentItem)
        {
        #ifdef USE_WIFI
            case kSetWifiMode:
                pushScreen(kWiFiModeScreen);
                break;
        #endif
            case kCalibrateLifter:
                pushScreen(kCalibrateScreen);
                break;
            case kEraseAllSettings:
                pushScreen(kEraseSettingsScreen);
                break;
        }
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SettingsScreen sSettingsScreen;
