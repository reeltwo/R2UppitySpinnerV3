///////////////////////////////////////////////////////////////////////////////
//
// Top level selection menu
// 
///////////////////////////////////////////////////////////////////////////////

static const char* sMainMenu[] = {
    "Select\nSequence",
    "Periscope\nUp",
    "Periscope\nDown",
    "Rotate\nPeriscope",
    "Rotate\nLeft",
    "Rotate\nRight",
    "Rotate\nStop",
    "Rotate\nForward",
    "Lights\nOff",
    "Lights\nCycle",
    "Lights\nObi-Wan",
    "Lights\nYoda",
    "Lights\nSith",
    "Lights\nSearch",
    "Lights\nDagobah",
    "Lights\nRandom",
    "Change\nSettings"
};

class MainScreen : public MenuScreen
{
public:
    enum {
        kSelectSequence,
        kPeriscopeUp,
        kPeriscopeDown,
        kRotatePeriscope,
        kRotateLeft,
        kRotateRight,
        kRotateStop,
        kRotateForward,
        kLightsCycle,
        kLightsOff,
        kLightsObiWan,
        kLightsYoda,
        kLightsSith,
        kLightsSearch,
        kLightsDagobah,
        kLightsRandom,
        kChangeSettings
    };
    MainScreen() :
        MenuScreen(kMainScreen, sMainMenu, SizeOfArray(sMainMenu))
    {}

#ifdef USE_DROID_REMOTE
    virtual void buttonLeftPressed(bool repeat) override
    {
        if (remoteEnabled)
        {
            DisconnectRemote();
        }
    }
#endif

    virtual void buttonInReleased() override
    {
        switch (fCurrentItem)
        {
            case kSelectSequence:
                pushScreen(kSelectScreen);
                break;
            case kPeriscopeUp:
                executeCommand(":PP100");
                break;
            case kPeriscopeDown:
                executeCommand(":PH");
                break;
            case kRotatePeriscope:
                pushScreen(kRotatePeriscopeScreen);
                break;
            case kRotateLeft:
                executeCommand(":PR50");
                break;
            case kRotateRight:
                executeCommand(":PR-50");
                break;
            case kRotateStop:
                executeCommand(":PD0");
                break;
            case kRotateForward:
                executeCommand(":PA0");
                break;
            case kLightsCycle:
                executeCommand(":PL0");
                break;
            case kLightsOff:
                executeCommand(":PL1");
                break;
            case kLightsObiWan:
                executeCommand(":PL2");
                break;
            case kLightsYoda:
                executeCommand(":PL3");
                break;
            case kLightsSith:
                executeCommand(":PL4");
                break;
            case kLightsSearch:
                executeCommand(":PL5");
                break;
            case kLightsDagobah:
                executeCommand(":PL6");
                break;
            case kLightsRandom:
                executeCommand(":PL7");
                break;
            case kChangeSettings:
                pushScreen(kSettingsScreen);
                break;
        }
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

MainScreen sMainScreen;
