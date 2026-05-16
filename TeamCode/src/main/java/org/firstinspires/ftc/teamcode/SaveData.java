package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class SaveData {

    private static final String SHARED_PREFERENCES = "DECODE.preferences";
    private final SharedPreferences sharedPreferences;

    public SaveData(HardwareMap hwMap) {
        sharedPreferences = hwMap.appContext.getSharedPreferences(SHARED_PREFERENCES, 0);
    }

    public double getDouble(SaveKey saveKey, double defaultValue) {
        return sharedPreferences.getFloat(saveKey.name(), (float)defaultValue);
    }

    public void putDouble(SaveKey saveKey, double value) {
        sharedPreferences.edit().putFloat(saveKey.name(), (float)value).apply();
    }

    public Alliance getAlliance(SaveKey saveKey, Alliance defaultValue) {
        String value = sharedPreferences.getString(saveKey.name(), null);
        if (value == null) return defaultValue;
        try {
            return Alliance.valueOf(value);
        } catch (IllegalArgumentException e) {
            return defaultValue;
        }
    }

    public void putAlliance(SaveKey saveKey, Alliance value) {
        sharedPreferences.edit().putString(saveKey.name(), value.name()).apply();
    }
}
