import send from "./../iotConnection"


function direction_factory(a,b,time1,time2){
    return function(){
      return new Promise(function(resolve, reject)
      {
        var interval = setInterval(function () {send(a,b);}, time1);
        setTimeout(function(){
          clearInterval(interval);
          send(0,0)
          send(0,0)
          send(0,0)
          resolve()
        }, time2);
      })
      }
    }

function loop(input, inside_c, rfuntion){
  console.log(inside_c)
  var	n = parseInt(input);
  for  (let i = 0; i < n; i++){
      rfuntion(inside_c)
    }
  }

// function sensor (input, inside_c,rfuntion){
//   while (input > 3){
//
//   }
//
//   run();
//   rfuntion(inside_c)
//
//
// }


const left = direction_factory(0,1,24,3500)
const right = direction_factory(0,-1,24,3500)
const forward = direction_factory(1,0,100,1000)
const reverse = direction_factory(-1,0,100,1000)


function get_function(command){
    const dict={
    "L" : left,
    "R" :right,
    "X":loop ,
    "F":forward,
    "B":reverse }
  return dict[command];

}

export default get_function;
