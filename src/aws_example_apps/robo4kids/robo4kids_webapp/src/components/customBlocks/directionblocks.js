import Blockly from 'blockly';
import 'blockly/javascript';
//import direction_factory from './direction_factory'


Blockly.Blocks['robot_direcction'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Move");
    this.appendDummyInput()
        .appendField(new Blockly.FieldDropdown([["Left","Left"], ["Right","Right"]]), "NAME");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['robot_forward'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Foward");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['robot_reverse'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Reverse");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['robot_loop'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Loop");
    this.appendValueInput("n")
        .setCheck("Number");
    this.appendStatementInput("internal_code")
        .setCheck(null);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['robot_laser'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("if Laser sensor");
    this.appendStatementInput("internal_code")
        .setCheck(null)
        .appendField("do:");
    this.setColour(230);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

//TODO: Change leters to words, parsing code is already done

Blockly.JavaScript['robot_direcction'] = function(block) {
  var dropdown_name = block.getFieldValue('NAME');
  // TODO: Assemble JavaScript into code variable.
  if (dropdown_name =="Left"){
    return "L,";
  }
  else{
    return 'R,';
  }
};


Blockly.JavaScript['robot_forward'] = function(block) {
  // TODO: Assemble JavaScript into code variable.
  var code = 'F,';
  return code;
};

Blockly.JavaScript['robot_reverse'] = function(block) {
  // TODO: Assemble JavaScript into code variable.
  var code = 'B,';
  return code;
};

Blockly.JavaScript['robot_loop'] = function(block) {
  var value_n = Blockly.JavaScript.valueToCode(block, 'n', Blockly.JavaScript.ORDER_ATOMIC);
  var statements_internal_code = Blockly.JavaScript.statementToCode(block, 'internal_code');
  // TODO: Assemble JavaScript into code variable.
  let L = statements_internal_code.trim().length+2
  var code = 'X,'+value_n+','+L+',('+statements_internal_code.trim()+ ')';
  console.log(statements_internal_code, code)
  return code;
};


Blockly.JavaScript['robot_laser'] = function(block) {
  var statements_internal_code = Blockly.JavaScript.statementToCode(block, 'internal_code');
  // TODO: Assemble JavaScript into code variable.
  let L = statements_internal_code.trim().length+2
  var code = 'L,'+0+','+L+',('+statements_internal_code.trim()+ ')';
  return code;
};
