import React, { useState } from 'react'
import Amplify from 'aws-amplify';
import awsconfig from './../aws-exports';
import awsiot from './../aws-iot';


import '@aws-amplify/ui/dist/style.css';

import { PubSub, Auth } from 'aws-amplify';
import { AWSIoTProvider } from '@aws-amplify/pubsub/lib/Providers';


//import direction_factory from '../customBlocks/direction_factory'

Amplify.configure(awsconfig);
Amplify.addPluggable(new AWSIoTProvider(awsiot));
//
//


Amplify.configure(awsconfig);
async function send(linear, angular){
    var rawText = {
        linear: {
            x: linear,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: angular
        }}
    await PubSub.publish('joystick1', rawText);
    console.log('Sending1: ' , rawText)
  }



export default send;
