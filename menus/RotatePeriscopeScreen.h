///////////////////////////////////////////////////////////////////////////////
//
// Allow manual rotation of dome for testing
// 
///////////////////////////////////////////////////////////////////////////////

class RotatePeriscopeScreen : public CommandScreen
{
public:
    RotatePeriscopeScreen(ScreenID id = kRotatePeriscopeScreen) :
        CommandScreen(sDisplay, id)
    {
        setKeyRepeatRate(50);
    }

    virtual void init() override
    {
        fLastDisplayPos = -1;
        fSavedOldValue = false;
    }

    virtual void render() override
    {
        if (millis() > fLastScreenUpdate + 100 || sDisplay.needsRedisplay())
        {
            int16_t pos = lifter.rotaryMotorCurrentPosition();
            if (pos != fLastPos)
                fLastPos = pos;
            if (fLastPos != fLastDisplayPos)
            {
                sDisplay.invertDisplay(false);
                sDisplay.clearDisplay();
                sDisplay.setTextSize(4);
                sDisplay.drawTextCentered(String(fLastPos));
                sDisplay.display();
                fLastDisplayPos = fLastPos;
            }
            fLastScreenUpdate = millis();
        }
    }

    virtual void buttonUpPressed(bool repeat) override
    {
        setRelativeTarget(90);
    }

    virtual void buttonLeftPressed(bool repeat) override
    {
        setRelativeTarget(10);
    }

    virtual void buttonDownPressed(bool repeat) override
    {
        setRelativeTarget(-90);
    }

    virtual void buttonRightPressed(bool repeat) override
    {
        setRelativeTarget(-10);
    }

    virtual void buttonDial(long newValue, long oldValue) override
    {
        if (!fSavedOldValue)
        {
            fOldValue = oldValue;
            fSavedOldValue = true;
        }
        long target = (long)fmod(newValue-fOldValue, 24) * 15;
        if (target < 0)
            target += 360;
        setTarget(target);
    }

    virtual void buttonInReleased() override
    {
        popScreen();
    }

protected:
    uint32_t fLastScreenUpdate = 0;
    int16_t fLastPos = -1;
    int16_t fLastDisplayPos = -1;
    bool fSavedOldValue = false;
    long fOldValue = 0;

    void setTarget(long target)
    {
        char buffer[20];
        // Avoid slow homing animation
        if (target == 0)
            target = 1;
        snprintf(buffer, sizeof(buffer), ":PA%d,50", int(target));
        executeCommand(buffer);
    }

    void setRelativeTarget(long relative)
    {
        setTarget((long)lifter.rotaryMotorCurrentPosition() + relative);
    }
};

///////////////////////////////////////////////////////////////////////////////
//
// Instantiate the screen
//
///////////////////////////////////////////////////////////////////////////////

RotatePeriscopeScreen sRotatePeriscopeScreen;
