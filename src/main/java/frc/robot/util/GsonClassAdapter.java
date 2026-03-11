package frc.robot.util;

import java.io.IOException;

import com.google.gson.JsonParseException;
import com.google.gson.TypeAdapter;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonWriter;

/** Gson adapter to dangerously serialize all classes. Used for record serialization. */
public class GsonClassAdapter extends TypeAdapter<Class<?>> {
    @Override
    public void write(JsonWriter out, Class<?> value) throws IOException {
        if (value == null) {
            out.nullValue();
        } else {
            out.value(value.getName()); // or getCanonicalName()
        }
    }

    @Override
    public Class<?> read(JsonReader in) throws IOException {
        String name = in.nextString();
        try {
            return Class.forName(name);
        } catch (ClassNotFoundException e) {
            throw new JsonParseException(e);
        }
    }
}