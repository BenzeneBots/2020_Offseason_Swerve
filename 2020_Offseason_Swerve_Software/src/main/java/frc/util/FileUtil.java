package frc.util;

import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.lang.Double;

public class FileUtil {

    public static void writeObjectToFile(File fileName, Object fill){
        try (RandomAccessFile raf = new RandomAccessFile(fileName, "rw")) {
            // set the file pointer at 0 position
            raf.seek(0);

            // write an int in the file
            if(fill instanceof Double || fill instanceof Integer){
                raf.writeDouble((double)fill);
            }else if(fill instanceof String){
                raf.writeBytes((String)fill);
            }else if(fill instanceof Boolean){
                raf.writeBoolean((boolean)fill);
            }
            // set the file pointer at 0 position
            raf.seek(0);

            raf.close();

        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    public static Object readObjectFromFile(Object fill,File fileName) {
        Object fetchVal = 0;
        try (RandomAccessFile raf = new RandomAccessFile(fileName, "rw")) {

            // set the file pointer at 0 position
            raf.seek(0);

            if(fill instanceof Double){
                fetchVal= raf.readDouble();
            }else if(fill instanceof Integer){
                fetchVal = raf.readInt();
            }else if(fill instanceof String){
                fetchVal = raf.readLine();
            }else if(fill instanceof Boolean){
                fetchVal = raf.readBoolean();
            }

            // set the file pointer at 0 position
            raf.seek(0);

            raf.close();
            
        } catch (IOException ex) {
            ex.printStackTrace();
        }
        return fetchVal;
    }
}
