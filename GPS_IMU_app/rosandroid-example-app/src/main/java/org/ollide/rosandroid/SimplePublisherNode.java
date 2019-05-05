/*
 * Copyright (C) 2014 Oliver Degener.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ollide.rosandroid;

import android.content.Context;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;

import org.ros.android.RosActivity;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;

import std_msgs.Bool;
import std_msgs.Float32MultiArray;
import std_msgs.Float64MultiArray;

import static android.content.ContentValues.TAG;
import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

public class SimplePublisherNode extends AbstractNodeMain implements NodeMain, MessageListener<Float64MultiArray> {

    private Boolean mode = TRUE;
    private Publisher<std_msgs.Float64MultiArray> publisher_GPS;
    private Publisher<std_msgs.Float64MultiArray> publisher_combined;
    private Publisher<std_msgs.Float32MultiArray> publisher_IMU;
    private Subscriber<std_msgs.Float64MultiArray> pos_subscriber;
    private Subscriber<std_msgs.Float64MultiArray> targets_subscriber;
//    int init_val = -1;
    public targetCordListener tg_obj=new targetCordListener();
    public boolean toggle;
    public boolean target_rcvd=FALSE;
    public double[] GPS_arr = {19,71};
    public double[] combined_arr = {19,71,0};
    public float[] Orientation_arr={0,0,0};
    public double[] message_data = {0,0,0};
    public double[] target_cords={0,0};
    public int GPS_updated=0;
    public int prev_GPS_updated=0;
    private ConnectedNode cNode;
    @Override
    public GraphName getDefaultNodeName() {
        if(mode){
            return GraphName.of("SimplePublisher/GPSIMU_sub");
        }

        else{
            return GraphName.of("SimplePublisher/GPSIMU_pub");
        }
    }

    @Override
    public void onNewMessage(final std_msgs.Float64MultiArray message) {
        message_data=message.getData();
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        if(!mode) {
            publisher_GPS = connectedNode.newPublisher(GraphName.of("LatLon"), std_msgs.Float64MultiArray._TYPE);
            publisher_IMU = connectedNode.newPublisher(GraphName.of("IMU"), std_msgs.Float32MultiArray._TYPE);
            publisher_combined = connectedNode.newPublisher(GraphName.of("test"), std_msgs.Float64MultiArray._TYPE);
        }
        else {
            message_data[0]=19.123442;
            publisher_GPS = connectedNode.newPublisher(GraphName.of("target_LatLon"), std_msgs.Float64MultiArray._TYPE);
            pos_subscriber = connectedNode.newSubscriber(GraphName.of("test"), std_msgs.Float64MultiArray._TYPE);
            targets_subscriber = connectedNode.newSubscriber(GraphName.of("targets"), std_msgs.Float64MultiArray._TYPE);
            pos_subscriber.addMessageListener(this);
            targets_subscriber.addMessageListener(tg_obj);
        }
        cNode=connectedNode;

        final CancellableLoop loop = new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {

                if(!mode) {
                    std_msgs.Float64MultiArray GPS_msg = publisher_GPS.newMessage();
                    std_msgs.Float32MultiArray IMU_msg = publisher_IMU.newMessage();
                    std_msgs.Float64MultiArray combined_msg = publisher_combined.newMessage();

                    IMU_msg.setData(Orientation_arr);
                    GPS_msg.setData(GPS_arr);
                    publisher_GPS.publish(GPS_msg);
                    publisher_IMU.publish(IMU_msg);

                    if(prev_GPS_updated!=GPS_updated)
                    {
                     combined_arr[0]=GPS_arr[0];
                     combined_arr[1]=GPS_arr[1];
                     combined_arr[2]=Orientation_arr[2];
                     combined_msg.setData(combined_arr);
                     publisher_combined.publish(combined_msg);
                     prev_GPS_updated=GPS_updated;
                    }

                }
                else{
                    std_msgs.Float64MultiArray GPS_msg = publisher_GPS.newMessage();
                    GPS_msg.setData(target_cords);
                    publisher_GPS.publish(GPS_msg);

                }
                //publisher_theta.publish(Theta_msg);
                // go to sleep for one second
                Thread.sleep(500);
                toggle=TRUE;
            }
        };
        connectedNode.executeCancellableLoop(loop);
    }


}
