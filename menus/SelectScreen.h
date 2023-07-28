#include <stdlib.h>

///////////////////////////////////////////////////////////////////////////////
//
// Sequence selection
// 
///////////////////////////////////////////////////////////////////////////////

class SelectScreen : public CommandScreen
{
public:
    SelectScreen() :
        CommandScreen(sDisplay, kSelectScreen)
    {}

    virtual void init()
    {
        fDisplayValue = ~0u;
        fNumValues = sSettings.getCommands(fValues, sizeof(fValues));
        qsort(fValues, fNumValues, 1, sequenceCompare);
        if (fNumValues > 0)
            fValue = 0;
    }

    virtual void buttonUpPressed(bool repeat) override
    {
        unsigned val = getValueToIndex(fValue);
        if (val > 0)
            fValue = fValues[val-1];
    }

    virtual void buttonLeftPressed(bool repeat) override
    {
        popScreen();
    }

    virtual void buttonDownPressed(bool repeat) override
    {
        unsigned val = getValueToIndex(fValue);
        if (val+1 < fNumValues)
        {
            fValue = fValues[val+1];
        }
    }

    virtual void buttonInReleased() override
    {
        if (fDisplayValue != ~0u)
        {
            popScreen();

            char buffer[10];
            snprintf(buffer, sizeof(buffer), ":PS%d", fValue);
            executeCommand(buffer);
        }
        else
        {
            popScreen();
        }
    }

    virtual void render() override
    {
        if (fValue != fDisplayValue || sDisplay.needsRedisplay())
        {
            sDisplay.clearDisplay();
            sDisplay.setTextSize(4);
            if (fValue == -2)
            {
                sDisplay.invertDisplay(true);
                sDisplay.drawTextCentered("N/A");
            }
            else
            {
                char buffer[10];
                snprintf(buffer, sizeof(buffer), ":PS%d", fValue);
                sDisplay.drawTextCentered(buffer);
            }
            sDisplay.display();
            fDisplayValue = fValue;
        }
    }

protected:
    int fDisplayValue = ~0;
    int fValue = -2;
    size_t fNumValues = 0;
    uint8_t fValues[256];

    unsigned getValueToIndex(unsigned value)
    {
        for (unsigned i = 0; i < fNumValues; i++)
        {
            if (fValues[i] == value)
                return i;
        }
        return 0;
    }

    static int sequenceCompare(const void* a, const void* b)
    {
        return int(*(uint8_t*)a) - int(*(uint8_t*)b);
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

SelectScreen sSelectScreen;
