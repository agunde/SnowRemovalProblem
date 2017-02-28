import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Created by andershg on 13.02.2017.
 */
public class Filewriter {

    private File file;
    private FileWriter writer;
    private BufferedWriter fw;

    public Filewriter(String fileName) {
        try{
            file = new File(fileName);

            writer = new FileWriter(file,true);
            fw  = new BufferedWriter(writer);

        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void writeTestOutput(String s) {
        //
        try {
            fw.write(s);
            fw.newLine();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
    public void flush(){
        try {
            fw.flush();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
