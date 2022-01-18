import './../../App.css';
import './../customBlocks/directionblocks';
import React, { useState, useEffect }  from 'react';
import ReactBlockly from 'react-blockly';
import Blockly from 'blockly';
import { Progress } from 'reactstrap';
import { Button, Modal } from 'react-bootstrap';
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


function Level2(props) {
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
//    document.getElementById('generated-xml').innerText = newXml;

    const code = Blockly.JavaScript.workspaceToCode(workspace);
    setCode(code);
  }

  useEffect(() =>
   //human_detection
     PubSub.subscribe('jetbot_msg/camera').subscribe({
     next: data => {
       try{
         setSensor(data.value)
         console.log(data.value);
       }
       catch (error){
         console.log("Error, are you sending the correct data?");
       }
     },
     error: error => console.error(error),
     close: () => console.log('Done'),
   }),[]
 );

  function Challenge(){
    const [show, setShow] = useState(false);

    const handleClose = () => setShow(false);
    const handleShow = () => setShow(true);
    if (sensor.message === "Human"){
      return (
        <div>
          <div className="text-center">100%</div>
          <Progress animated color="success" value="100" />
          <div class="d-grid gap-2 d-md-flex justify-content-md-end">
          <Button variant="info" size="sm" onClick={handleShow}>
            Finish!
          </Button>
          </div>
          <Modal show={show} onHide={handleClose} backdrop="static" keyboard={false}>
            <Modal.Header closeButton>
              <Modal.Title>Congratulations My Friend!</Modal.Title>
            </Modal.Header>
            <Modal.Body>
              AWSome you did it!! Now, Please go back to have more fun!!
              <img
                className="d-block w-100"
                src="dolphin.jpg"
                alt="Error Loading the Image"
              />
            </Modal.Body>
            <Modal.Footer>
              <Button variant="secondary" onClick={handleClose}>
                Close
              </Button>
              <Button variant="primary" onClick={props.menu}>Go Back</Button>
            </Modal.Footer>
          </Modal>
        </div>
      )
    } else {
      return (
        <div>
          <div className="text-center">0%</div>
          <Progress />
        </div>
      )
    }
  }

  return (
    <div id="blocklyArea">
      <ReactBlockly
        toolboxCategories={toolboxCategories}
        wrapperDivClassName="blocklyDiv"
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
      <div id="runButton">
        <RunCode code={code}/>
        <Challenge/>
      </div>
    </div>
  )
}

export default Level2;
