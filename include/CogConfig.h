#pragma once

class CogConfig
{
  private:
    static CogConfig _instance;
    CogConfig()
    {
        showVisuals = false;
        showAgentInfo = false;
        showAgentSelectedOnly = false;
    }

  public:
    static CogConfig *getInstance()
    {
        return &_instance;
    }

    bool showVisuals;
    bool showAgentInfo;
    bool showAgentSelectedOnly;
};

