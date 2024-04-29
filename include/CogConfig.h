#pragma once

class CogConfig
{
  private:
    CogConfig() { showVisuals = false; }

  public:
    static CogConfig *getInstance()
    {
        static CogConfig _instance;
        return &_instance;
    }

    bool showVisuals;
};
