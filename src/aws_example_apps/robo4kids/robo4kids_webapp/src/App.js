import './App.css';
import './components/customBlocks/directionblocks'
import React, { useState, useEffect }  from 'react'
import ReactBlockly from 'react-blockly'
import Blockly from 'blockly';
import Button from 'react-bootstrap/Button';
import Col from 'react-bootstrap/Col';
import Row from 'react-bootstrap/Row';


import RunCode from './components/runCode';

import Amplify from 'aws-amplify';
import awsconfig from './aws-exports';
import { withAuthenticator } from 'aws-amplify-react'; // or 'aws-amplify-react-native';
import '@aws-amplify/ui/dist/style.css';
import { PubSub, Auth } from 'aws-amplify';
import { AWSIoTProvider } from '@aws-amplify/pubsub/lib/Providers';

import Level1 from './components/levels/level1';
import Level2 from './components/levels/level2';
import Level3 from './components/levels/level3';
import Level0 from './components/levels/level0';
import GetInstructions from './components/instructions/instructions.js'


Amplify.configure(awsconfig);

// Amplify.addPluggable(new AWSIoTProvider({
//  aws_pubsub_region: 'us-west-2',
//  aws_pubsub_endpoint: 'wss://ajlhtk9kwdt15-ats.iot.us-west-2.amazonaws.com/mqtt',
// }));

function App() {
  const [level, setLevel] = useState(-1);


  function level_1(){
    setLevel(1);
  };
  function level_2(){
    setLevel(2);
  };
  function level_3(){
    setLevel(3);
  };
  function level_0(){
    setLevel(0);
  };
  function menu(){
    setLevel(-1);
  };

  function Render_level(props){
    const level = props.level;
    if (level==1){
      return(<Level1 menu={props.menu}/>)
    }
    if (level==2){

      return( <Level2 menu={props.menu}/>)
    }
    if (level==3){

      return( <Level3 menu={props.menu}/>)
    }
    if (level==0){
      return( <Level0 menu={props.menu}/>)
    }
    else{
      return(
        <div id="lvlSelection">

          <div id="introLevels" class="jumbotron">
            <h1 class="display-4">Start learning robotics!</h1>
            <p class="lead">You don't need to know anything about programming and still will be able to control and add features to a robot and have fun.</p>
            <hr class="my-4"></hr>
            <p class="lead">Choose a level to start:</p>
          </div>
          <div id="lvlOptions">
            <table id="detailOptions">
              <th></th>
              <th>
                <tr class="buttonRow">
                  <Button onClick={level_0} > Free Level </Button>
                </tr>
                <tr class="titleRow">
                  EMPTY LEVEL
                </tr>
                <tr class="detailsRow">
                  <p>Explore and interact with the robot using all blocks available</p>
                </tr>
              </th>
              <th></th>

            </table>
          </div>

          <div id="lvlOptions">
            <table id="detailOptions">
              <th>
                <tr class="buttonRow">
                  <Button disabled onClick={level_1} > Level 1 </Button>
                </tr>
                <tr class="titleRow">
                  Comming Soon!
                </tr>
                <tr class="titleRow">
                  THE MAZE
                </tr>
                <tr class="detailsRow">
                  <p>Control a robot through the maze and find the mailbox to deliver an important letter!</p>
                </tr>
              </th>
              <th>
                <tr class="buttonRow">
                  <Button disabled onClick={level_2} > Level 2 </Button>
                </tr>
                <tr class="titleRow">
                  Comming Soon!
                </tr>
                <tr class="titleRow">
                  THE HOUSE
                </tr>
                <tr class="detailsRow">
                  <p>Control a robot through a small house and help the robot to identify the humans!</p>
                </tr>
              </th>
              <th style={{color:'#a6a6a6'}}>
                <tr class="buttonRow">
                  <Button disabled onClick={level_3} > Level 3 </Button>
                </tr>
                <tr class="titleRow">
                  Comming Soon!
                </tr>
                <tr class="titleRow"  style={{fontSize:'20px',letterSpacing: '0px'}}>
                </tr>
                <tr class="detailsRow" style={{fontSize:'15px'}}>
                  <p>New levels and worlds to keep learning!</p>
                </tr>
              </th>

            </table>
          </div>
        </div>
      )
    }
  }


  return (
    <div id="levelArea" class="row">
      <div id="blocklyArea" class="col nopadding">
        <Render_level level={level}  menu={menu}/ >
      </div>
      <div id="instructionsArea" class="col-auto nopadding">
        <GetInstructions level={level}/>
      </div>
    </div>

  )
}

export default App;
