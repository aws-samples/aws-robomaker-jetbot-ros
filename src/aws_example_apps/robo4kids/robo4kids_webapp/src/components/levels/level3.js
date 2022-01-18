import './../customBlocks/directionblocks'
import React, { useState, useEffect }  from 'react'
import ReactBlockly from 'react-blockly'
import Blockly from 'blockly';

//
// import 'bootstrap/dist/css/bootstrap.min.css';
// import Container from 'react-bootstrap/Container';
// import Row from 'react-bootstrap/Row';
// import Col from 'react-bootstrap/Col'

import RunCode from './../runCode';

import Amplify from 'aws-amplify';
import awsconfig from './../../aws-exports';
import { withAuthenticator } from 'aws-amplify-react'; // or 'aws-amplify-react-native';
import '@aws-amplify/ui/dist/style.css';
import { PubSub, Auth } from 'aws-amplify';
import { AWSIoTProvider } from '@aws-amplify/pubsub/lib/Providers';

Amplify.configure(awsconfig);



function Level3() {
  //const initialXml = '<xml xmlns="http://www.w3.org/1999/xhtml"><block type="text" x="70" y="30"><field name="TEXT"></field></block></xml>';
  const toolboxCategories = [
    {
      name: 'Direcction',
      colour: '#5CA699',
      blocks: [
        {
          type: 'robot_direcction'
        },
        {
          type: 'robot_forward'
        },
        {
          type: 'robot_reverse'
        },

        {
          type: 'robot_loop'
        },
        {
          type: 'math_number'
        },

      ]
    },


  ]
  const [code, setCode] = useState('');
  const [sensor, setSensor] = useState('');
  function workspaceDidChange(workspace) {
    const newXml = Blockly.Xml.domToText(Blockly.Xml.workspaceToDom(workspace));
  //  document.getElementById('generated-xml').innerText = newXml;

    const code = Blockly.JavaScript.workspaceToCode(workspace);
    setCode(code);
  }

   useEffect(() =>
   //human_detection
     PubSub.subscribe('').subscribe({
     next: data => {
       try{
         setSensor(data.value)
       }
       catch (error){
         console.log("Error, are you sending the correct data?");
       }
     },
     error: error => console.error(error),
     close: () => console.log('Done'),
   }),[]
 );


  return (
    <div>
      <ReactBlockly
        toolboxCategories={toolboxCategories}
        wrapperDivClassName="fill-height"
        workspaceConfiguration={{
          grid: {
            spacing: 20,
            length: 3,
            colour: '#ccc',
            snap: true,
          },
        }}
        workspaceDidChange={workspaceDidChange}
      />

      <RunCode code={code}/>
      <p> {sensor.message}</p>

    </div>
  )
}

export default Level3;
