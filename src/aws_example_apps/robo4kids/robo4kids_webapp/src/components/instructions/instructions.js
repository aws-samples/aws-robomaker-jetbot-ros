import './../../App.css';
import React, { useState, useEffect }  from 'react'
import Button from 'react-bootstrap/Button';
import Collapse from 'react-bootstrap/Collapse';
import Col from 'react-bootstrap/Col';
import Row from 'react-bootstrap/Row';
import Image from 'react-bootstrap/Image';



function GetInstructions(props){
  const level = props.level;
  const [open, setOpen] = useState(false);

  if (level==1){
    return(
      <div id="instructionsContent">
        <Button
          class="btn btn-light"
          style={{backgroundColor: "#f8f8fa", border:"none"}}
          onClick={() => setOpen(!open)}
          aria-controls="instructionsText"
          aria-expanded={open}
        >
          <img style={{width:"20px"}} src="help-icon.png"/>
        </Button>
        <Collapse in={open}>
          <div id="instructionsText">
            <Col>
              <Row>
                <ul class="list-unstyled">
                  <li class="list-title">The Maze</li>
                  <li style={{lineHeight:"50px"}}><em>Find the mailbox lost in the maze.</em></li>
                  <li><Image src="Maze.png" style={{width:"400px"}}/></li>
                  <li><hr style={{width:"600px"}}></hr></li>
                </ul>
              </Row>
              <Row>

                <img style={{width:"60px"}} />
              </Row>
              <Row>
                <ul class="list-unstyled">
                  <li>Control the robot using <strong>Direction</strong> blocks:
                    <ul>
                      <li>Move: You can select wheter you want to rotate.
                        <ul>
                          <li style={{fontSize:'12px'}}>Use the dropdown list to select the direction you</li>
                          <li class="list-unstyled"  style={{fontSize:'12px'}}>want the robot to rotate.</li>
                        </ul>
                      </li>
                      <li>Forward: Make the robot move forward one step.</li>
                      <li>Reverse: Make the robot move behind one step.</li>
                      <li>Loop: Repeat the sequence of blocks indefinitely.
                        <ul>
                          <li style={{fontSize:'12px'}}>Use the little number block to indicate how many times</li>
                          <li class="list-unstyled" style={{fontSize:'12px'}}>you want to repeat the actions inside the Loop block.</li>
                        </ul>
                      </li>
                    </ul>
                  </li>
                </ul>
                <p>Use the <strong>Run Code</strong> button to let know your robot the sequence of movements to do.</p>
              </Row>
            </Col>
          </div>
        </Collapse>
      </div>
      )
  }

  if (level==2){
    return(
      <div id="instructionsContent">
        <Button
          class="btn btn-light"
          style={{backgroundColor: "#f8f8fa", border:"none"}}
          onClick={() => setOpen(!open)}
          aria-controls="instructionsText"
          aria-expanded={open}
        >
          <img style={{width:"20px"}} src="help-icon.png"/>
        </Button>
        <Collapse in={open}>
          <div id="instructionsText" style={{padding: "10px"}}>
            <Col>
              <Row>
                <ul class="list-unstyled">
                  <li class="list-title">The House</li>
                  <li style={{lineHeight:"50px"}}><em>Find the people on the house.</em></li>
                  <li><Image src="small-house.png" style={{width:"400px"}}/></li>
                  <li><hr style={{width:"600px"}}></hr></li>
                </ul>
              </Row>
              <Row>
                <ul class="list-unstyled">
                  <li>Control the robot using <strong>Direction</strong> blocks:
                    <ul>
                      <li>Move: You can select wheter you want to rotate.
                        <ul>
                          <li style={{fontSize:'12px'}}>Use the dropdown list to select the direction you</li>
                          <li class="list-unstyled" style={{fontSize:'12px'}}>want the robot to rotate.</li>
                        </ul>
                      </li>
                      <li>Forward: Make the robot move forward one step.</li>
                      <li>Reverse: Make the robot move behind one step.</li>
                      <li>Loop: Repeat the sequence of blocks indefinitely.
                        <ul>
                          <li style={{fontSize:'12px'}}>Use the little number block to indicate how many times</li>
                          <li class="list-unstyled" style={{fontSize:'12px'}}>you want to repeat the actions inside the Loop block.</li>
                        </ul>
                      </li>
                    </ul>
                  </li>
                  <li>Control the sensor using <strong>Sensor</strong> blocks:
                    <ul>
                      <li>Block 1:
                        <ul>
                          <li><small>Description</small></li>
                          <li class="list-unstyled"><small>block 1</small></li>
                        </ul>
                      </li>
                      <li>Block 2:</li>
                      <li>Block 3:</li>
                    </ul>
                  </li>
                </ul>
                <p>Use the <strong>Run Code</strong> button to let know your robot the sequence of tasks to do.</p>
              </Row>
            </Col>
          </div>
        </Collapse>
      </div>
      )
  }
  if (level==3){

  }
  if (level==0){
    return(
    <div id="instructionsContent">
        <Button
          class="btn btn-light"
          style={{backgroundColor: "#f8f8fa", border:"none"}}
          onClick={() => setOpen(!open)}
          aria-controls="instructionsText"
          aria-expanded={open}
        >
          <img style={{width:"20px"}} src="help-icon.png"/>
        </Button>
        <Collapse in={open}>
          <div id="instructionsText">
            <Col>
                <ul class="list-unstyled">
                  <li>Control the robot using <strong>Direction</strong> blocks:
                    <ul>
                      <li>Move: You can select wheter you want to rotate.
                        <ul>
                          <li style={{fontSize:'12px'}}>Use the dropdown list to select the direction you</li>
                          <li class="list-unstyled"  style={{fontSize:'12px'}}>want the robot to rotate.</li>
                        </ul>
                      </li>
                      <li>Forward: Make the robot move forward one step.</li>
                      <li>Reverse: Make the robot move behind one step.</li>
                      <li>Loop: Repeat the sequence of blocks indefinitely.
                        <ul>
                          <li style={{fontSize:'12px'}}>Use the little number block to indicate how many times</li>
                          <li class="list-unstyled" style={{fontSize:'12px'}}>you want to repeat the actions inside the Loop block.</li>
                        </ul>
                      </li>
                    </ul>
                  </li>
                </ul>
                <p>Use the <strong>Run Code</strong> button to let know your robot the sequence of movements to do.</p>
            </Col>
          </div>
        </Collapse>
      </div>
    )
  }
  else{
    return(
      <div id="instructionsContent">
        <Button
          class="btn btn-light"
          style={{backgroundColor: "#f8f8fa", border:"none"}}
          onClick={() => setOpen(!open)}
          aria-controls="instructionsText"
          aria-expanded={open}
        >
          <img style={{width:"20px"}} src="help-icon.png"/>
        </Button>
        <Collapse in={open}>
          <div id="instructionsText" style={{padding: "10px"}}>
            <ul class="list-unstyled">
              <li>Select one level to get started.</li>
              <li></li>
              <li>Difficulty:
                <ul>
                  <li>Level 1 is a simple world where you will have to </li>
                  <li class="list-unstyled">control the movement of the robot.</li>
                  <li>Level 2 is a small house world where you will have to </li>
                  <li class="list-unstyled">control the movement and fine tuning the sensors </li>
                  <li class="list-unstyled">to find objects.</li>
                  <li>Level 3 is on construction.</li>
                  <li>Free level is on construction.</li>
                </ul>
              </li>
            </ul>
            <p class="list-title" style={{textAlign:'center'}}>Click to get started!</p>
          </div>
        </Collapse>
      </div>
    )
  }
}


export default GetInstructions;
