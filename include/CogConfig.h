#pragma once

class CogConfig {
private:
	CogConfig() {
		showVisuals = true;
	}

public:
	static CogConfig* getInstance() {
		static CogConfig _instance;
		return &_instance;
	}

	bool showVisuals;
};
