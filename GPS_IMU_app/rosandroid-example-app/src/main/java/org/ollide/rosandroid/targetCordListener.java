package org.ollide.rosandroid;

import org.ros.message.MessageListener;

import java.util.Arrays;

import std_msgs.Float64MultiArray;

import static java.lang.Boolean.FALSE;

import static java.lang.Boolean.TRUE;

/**
 * Created by akshit on 4/5/19.
 */

public class targetCordListener implements MessageListener<Float64MultiArray> {
    public double[] msg_data = new double[20];
//    public boolean ready = FALSE;
//    public void targetCordListener(int init_val){
//        for (int i = 0 ;i<20 ;i++)
//        {
//            msg_data[i]=init_val;
//        }
//        ready = TRUE;
//    }

    @Override
    public void onNewMessage(final std_msgs.Float64MultiArray message) {
        msg_data = message.getData();
        System.out.println(msg_data[0]);
    }
}
