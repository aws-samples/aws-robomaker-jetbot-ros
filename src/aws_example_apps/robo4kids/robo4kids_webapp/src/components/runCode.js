import React, { useState } from 'react'
import Amplify from 'aws-amplify';
import awsconfig from './../aws-iot';
import '@aws-amplify/ui/dist/style.css';
import Blockly from 'blockly';
import 'blockly/javascript';

import { PubSub, Auth } from 'aws-amplify';
import { AWSIoTProvider } from '@aws-amplify/pubsub/lib/Providers';

import 'bootstrap/dist/css/bootstrap.min.css';
import get_function from './customBlocks/direction_factory'



Amplify.configure(awsconfig);



function RunCode(props) {


  var list_funtions = []


  async function run()
    {

      //console.log(props.code);


      createList(props.code)
      console.log(list_funtions)
      window.LoopTrap = 100;
      Blockly.JavaScript.INFINITE_LOOP_TRAP = 'if(--window.LoopTrap == 0) throw "Infinity loop";\n'
      for await(let l of list_funtions) {
            await l();

          }
      list_funtions = []
      Blockly.JavaScript.INFINITE_LOOP_TRAP = null;
    }


  function createList(code)
        {
        console.log("run", code)
       	let i = 0;
      	while( i < code.length-1){

    		if(code[i]===',' || code[i]==='(' ||code[i]===')'){
    			i++
    			continue;
    		}

    		const f_position = code.substring(i,).indexOf(',') + i;
      		var comand = code.substring(i, f_position);
      		i = f_position;

      		if(comand[0]=== "X"){
            //Asolating the input
      			const fp_input = code.substring(i+1).indexOf(',') +i+1;
      			var input = parseInt(code.substring(i+1, fp_input));

      			//Asolating the length of statment
      			const fp_lenght = code.substring(fp_input+1).indexOf(',') +fp_input+1;
      			var length = parseInt(code.substring(fp_input+1, fp_lenght));

      			var inside_code = code.substring(fp_lenght+2,length+fp_lenght)
      			get_function(comand)(input,inside_code, createList);
      			i = length+fp_lenght;

      			console.log('input, length, inside_code',input,length, inside_code)

      		}


    		else{
    		list_funtions.push(get_function(comand));
    		}

    		i++
    	}


     }


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
    console.log('Sending3: to joystick')
    console.log('Sending2: ' + rawText)
  }




return(
  <>
    <button onClick={run} type="button" class="btn btn-warning btn-lg btn-block"> Run Code</button>
  </>
)

}

export default RunCode;
