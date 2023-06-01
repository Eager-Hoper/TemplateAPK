package test;

import android.util.Log;


public class testLogMessage{
//    public static final String TAG;

//    static {
//        TAG = testLogMessage.class.getSimpleName();
//    }

    public static void main(String[] args) {

        for (int i=1; i<=10; i++) {
            System.out.println(i);
        }
        Log.e("TAG", "hello");
    }
}