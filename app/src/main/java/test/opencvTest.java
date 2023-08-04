package test;
import org.opencv.core.Mat;
import org.opencv.core.MatVector;
import org.opencv.core.Scalar;
import org.opencv.core.highgui;
import org.opencv.core.opencv_imgcodecs.*;

public class ImageReader {

    public static void main(String[] args) {
        String imageFilePath = "sample.png";

        // 画像を読み込む
        Mat image = imread(imageFilePath);

        // 画像の幅と高さを取得
        int width = image.cols();
        int height = image.rows();
        System.out.println("Width: " + width + ", Height: " + height);

        // 画像の処理や表示などの操作をここで行う
        // 例: ウィンドウに画像を表示する
        opencv_highgui.imshow("Image", image);
        opencv_highgui.waitKey();
    }
}
