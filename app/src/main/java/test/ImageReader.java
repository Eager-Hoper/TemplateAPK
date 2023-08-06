package test;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.imgcodecs.Imgcodecs;
//import org.opencv.core.opencv_imgcodecs.*;

public class ImageReader {

    public static void main(String[] args) {
        String imageFilePath = "sample.png";

        //Loading the OpenCV core library
        System.out.println(Core.NATIVE_LIBRARY_NAME);
        System.loadLibrary( Core.NATIVE_LIBRARY_NAME );

        // 画像を読み込む
        Imgcodecs imageCodecs = new Imgcodecs();
        Mat image = imageCodecs.imread(imageFilePath);

        // 画像の幅と高さを取得
        int width = image.cols();
        int height = image.rows();
        System.out.println("Width: " + width + ", Height: " + height);

        // 画像の処理や表示などの操作をここで行う
        // 例: ウィンドウに画像を表示する
        // opencv_highgui.imshow("Image", image);
        // opencv_highgui.waitKey();
    }
}
