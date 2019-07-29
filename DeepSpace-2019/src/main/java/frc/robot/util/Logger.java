package frc.robot.util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

public class Logger {
    FileWriter fw;
    Map<String, Object> entries = new HashMap<>();
    boolean firstWrite = true;

    public Logger(String subSystem) {
        String path = "/home/lvuser/" + subSystem + ".csv";
        try {
            File f = new File(path);
            if (f.exists()) {
                f.delete();
            }
            fw = new FileWriter(f, true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void clear() {
        entries = new HashMap<>();
    }

    public void entry(String key, Object value) {
        entries.put(key, value);
    }

    public void writeEntries() {
        if (firstWrite) {
            String titles = entries.keySet().stream().collect(Collectors.joining(", "));
            try {
                fw.write(titles);
            } catch (IOException e) {
                e.printStackTrace();
            }
            firstWrite = false;
        }
        String values = entries.values().stream().map(Object::toString).collect(Collectors.joining(", "));
        try {
            fw.write(values);
        } catch (IOException e) {
            e.printStackTrace();
        }
}
}