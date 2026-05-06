package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Properties;

public class SaveData {

    private static final String PATH = Environment.getExternalStorageDirectory().getPath() + "/DECODE/";
    private static final String FILE_NAME = "DECODE.properties";

    private final Properties properties = new Properties();

    public SaveData(Telemetry telemetry) {
        try {
            Path path = Paths.get(PATH);
            File dir = new File(path.toUri());
            if (!dir.exists())
                dir.mkdirs();
            File file = new File(dir, FILE_NAME);
            properties.clear();
            properties.load(Files.newInputStream(file.toPath()));
        } catch (Exception ignored) {
            properties.clear();
        }
    }

    public double getDouble(SaveKey saveKey, double defaultValue) {
        Double value = (Double) properties.get(saveKey.name());
        return value == null ? defaultValue : value;
    }

    public void putDouble(SaveKey saveKey, double value) {
        properties.put(saveKey.name(), value);
    }

    public Alliance getAlliance(SaveKey saveKey, Alliance defaultValue) {
        String value = (String) properties.get(saveKey.name());
        if (value == null) return defaultValue;
        try {
            return Alliance.valueOf(value);
        } catch (IllegalArgumentException e) {
            return defaultValue;
        }
    }

    public void putAlliance(SaveKey saveKey, Alliance value) {
        properties.put(saveKey.name(), value.name());
    }

    public void save() {
        try {
            properties.store(Files.newOutputStream(Paths.get(PATH)), "properties");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
