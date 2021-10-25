<template>
  <div class="hello">
    <h1>{{ msg }}</h1>    
    <!-- <br> -->
    
      <!-- Lifecycle Node Control Dashboard<br> -->

      <div class="container">
  <div class="row justify-content-md-center">
      <div class="col-md-auto">
      <MDBInput
              class
              inputGroup
              :formOutline="false"
              v-model="input_node_name"
              aria-label="Example text with button addon"
              aria-describedby="button-addon1"
              placeholder="Node name"
              v-bind:style="[{ height: 20 }, {background: ''}]"
            >
              <template #prepend>
                <MDBBtn color="success" rounded v-on:click="add_node">Add Lifecycle Nodes + </MDBBtn>
              </template>
              
            </MDBInput>
    </div>
  </div>
  <div class="row justify-content-md-center">
    <div class="col-md-auto p-3 border">
      Added nodes:
          <MDBBtnGroup aria-label="Basic example" v-for="data in added_nodes" :key="data">
                <MDBBtn color="primary" size="lg" v-on:click="set_active_node(data)">   {{data}}   </MDBBtn>

          </MDBBtnGroup>
    </div>
  </div>
  <div class="row justify-content-md-center">
    <div class="col-md-auto p-3 border">
        Active Node:<MDBBtn color="success" class="active" > {{active_node}} </MDBBtn>
        <span :key="current_state"> Current State: {{current_state}} </span>
    </div>
  </div>
</div>
    <MDBContainer class="px-4">
      <MDBRow>
          <MDBCol>  
            <!-- <MDBInput
              class
              inputGroup
              :formOutline="false"
              v-model="input_node_name"
              aria-label="Example text with button addon"
              aria-describedby="button-addon1"
              placeholder="Node name"
              v-bind:style="[{ height: 20 }, {background: ''}]"
            >
              <template #prepend>
                <MDBBtn color="success" rounded v-on:click="add_node">Add Lifecycle Nodes + </MDBBtn>
              </template>
              
            </MDBInput> -->
      </MDBCol>
    </MDBRow>
  
     <MDBRow class="gx-5">
      <MDBCol>
        
        <div class="p-3">
                <!-- <button type="button" class="btn btn-primary" v-on:click="update">Update Data</button>
       <button type="button" class="btn btn-primary" v-on:click="clear">Clear Data</button> -->
      <button type="button" class="btn btn-primary" v-on:click="srvGetCurrentState">Call srvGetCurrentState</button>
      
      <button type="button" class="btn btn-primary" v-on:click="srvGetAvailableStates">Call srvGetAvailableStates</button> 
      <button type="button" class="btn btn-secondary" v-on:click="srvGetAvailableTransitions">Call srvGetAvailableTransitions</button> 
      
       <br>
        <br>
       <button type="button" class="btn btn-success" v-on:click="subscribeToRosOut">Enable Logger</button> 
      <button type="button" class="btn btn-primary" v-on:click="srvChangeState">State Transition</button> 
      <select v-model="selected_transition" v-bind:style="[{ height: '20' }, {background: 'gray'}]">
        
        <option v-for="option in available_transitions" :key="option" v-bind:value="option.transition.id" >
          {{ option.transition.label }}
        </option>
      </select>
         
        <hr>
        </div>
      </MDBCol>
    </MDBRow>
  </MDBContainer>
  
  
      <!-- <button type="button" class="btn btn-primary" v-on:click="update">Update Data</button>
       <button type="button" class="btn btn-primary" v-on:click="clear">Clear Data</button>
      <button type="button" class="btn btn-primary" v-on:click="srvGetCurrentState">Call srvGetCurrentState</button>
      
      <button variant="danger" v-on:click="srvGetAvailableStates">Call srvGetAvailableStates</button> 
      
       <br>
        <br>
      <button variant="danger" v-on:click="srvChangeState">Call srvChangeState</button> 
      <select v-model="selected_transition">
        <option v-for="option in c_available_transitions" :key="option" v-bind:value="option.transition.id" >
          {{ option.transition.label }}
        </option>
      </select>

      <span>Selected: {{ selected_transition }}</span>

      <br>
      <span :key="current_state"> Current State: {{current_state}} </span> -->
      <!-- <ul id="available_states">
        Available States:
        <li v-for="item in available_states" :key="item.id">
          {{ item.label }}
        </li>
      </ul> -->
  <div class="container">
  <div class="row justify-content-md-center">
    <div class="col-md-auto">
       <h5>Available Transitions:</h5>
    <table id="myTable1"  class="table table-light table-striped table-hover" v-bind:style="[{ height: 20 }, {background: ''}]">
      <thead class="table-dark"> 
        <tr>
          <th scope="col">transition</th>
          <th scope="col">start_state</th>
          <th scope="col">goal_state</th>
        </tr>
      </thead>
      <tbody> 
      <tr v-for="data in available_transitions" v-bind:key="data">
            <td> {{data.transition}}</td>
            <td> {{data.start_state}}</td> 
            <td> {{data.goal_state}}</td>     
      </tr>
      </tbody>
    </table>
    </div>
     <!-- <div class="col-md-auto">
      <h6>Available States:</h6>
    <table id="myTable2"  class="table table-light table-striped table-hover table-sm" width="50%">
      <thead class="table-dark">
        <tr>
          <th scope="col">Id</th>
          <th scope="col">Label</th>
        </tr>
      </thead>
      <tbody> 
      <tr v-for="data in available_states" :key="data.id">
            <td> {{data.id}}</td>   
            <td> {{data.label}}</td>  
      </tr>
      </tbody>
    </table>
    </div> -->
  </div>
  <div class="row justify-content-md-center">
    <div class="col-md-auto">
      <h6>Available States:</h6>
    <table id="myTable2"  class="table table-light table-striped table-hover table-sm" width="50%">
      <thead class="table-dark">
        <tr>
          <th scope="col">Id</th>
          <th scope="col">Label</th>
        </tr>
      </thead>
      <tbody> 
      <tr v-for="data in available_states" :key="data.id">
            <td> {{data.id}}</td>   
            <td> {{data.label}}</td>  
      </tr>
      </tbody>
    </table>
    </div>
    <div class="col-md-auto">
      <h6>Logger</h6>
      <table id="myTable3"  class="table table-dark table-borderless table-sm" width="50%">
      <thead class="table-info">
        <tr>
          <th scope="col">Select Logger Level:</th>
          <th>   <select v-model="selected_logger_level"  class="form-select" aria-label="Default select example">
                  <option selected>ALL</option>
                  <option value="40">DEBUG</option>
                  <option value="30">WARN</option>
                  <option value="20">INFO</option>
                </select> </th>

          <th scope="col">{{selected_logger_level}}</th>
        </tr>
      </thead>
      <tbody> 
      <tr v-for="data in console_out" :key="data">
          <td> {{mapLogLevel(data.level)}}</td> 
          <td> {{data.msg}}</td>    
      </tr>
      </tbody>
    </table>
    </div>
  </div>
</div>

    
    

  <button type="button" class="btn btn-info" v-on:click="unsub=true">Disable Logger: {{unsub}}</button>
  
    <!-- {{console_out}} -->

  
  </div>
</template>


<script>
// import * as rclnodejs from 'rclnodejs';
import * as ROSLIB from 'roslib';
import { MDBBtn, MDBBtnGroup, MDBInput, MDBContainer, MDBRow, MDBCol } from "mdb-vue-ui-kit";
// import VirtualList from 'vue-virtual-scroll-list'
// import VueScrollingTable from "vue-scrolling-table"


// import * as ROSLIB from '../assets/js/roslib.js';
// const rclnodejs = require('rclnodejs');
// rclnodejs.init().then(() => {
//       const node = new rclnodejs.Node('publisher_example_node');
//       const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
//       publisher.publish(`Hello ROS 2 from rclnodejs`);
//       node.spin();
//     }

export default {
  
  name: 'HelloWorld',
  components: {
    MDBBtn, MDBBtnGroup, MDBInput, MDBContainer, MDBRow, MDBCol
  },
  props: {
    msg: String,
  },
  data: function () {
    return {
      unsub: false,
      console_out: [],
      selected_logger_level: 20,
      input_node_name:"",
      added_nodes:["lifecycle_joytovel"],
      active_node: "lifecycle_joytovel",
      selected_transition: {},
      available_states: [],
      available_transitions: [],
      current_state: {},
      ros: new ROSLIB.Ros({
        url : 'ws://localhost:9090'
      })
    }
  },
  
  mounted: function () {
    console.log("mounted")
    // var _ros = new ROSLIB.Ros({});
    // _ros.connect('ws://127.0.0.1:9090');

     var ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
      });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  this.update();
  // this.srvGetAvailableStates();
    // let rclnode = new rclnodejs();
    
    // console.log(rclnodejs.init());
    // rclnodejs.init().then(() => {
    //   const node = new rclnodejs.Node('publisher_example_node');
    //   const publisher = node.createPublisher('std_msgs/msg/String', 'topic');
    //   publisher.publish(`Hello ROS 2 from rclnodejs`);
    //   node.spin();
    // }
  },
  methods: {

    // Available Services :
    // /lifecycle_joytovel/change_state
    // /lifecycle_joytovel/describe_parameters
    // /lifecycle_joytovel/get_available_states
    // /lifecycle_joytovel/get_available_transitions
    // /lifecycle_joytovel/get_parameter_types
    // /lifecycle_joytovel/get_parameters
    // /lifecycle_joytovel/get_state
    // /lifecycle_joytovel/get_transition_graph
    // /lifecycle_joytovel/list_parameters
    // /lifecycle_joytovel/set_parameters
    // /lifecycle_joytovel/set_parameters_atomically
    clear() {
      this.available_states = [];
      this.current_state = [];
      this.available_transitions = [];
    },
    update() {
      this.srvGetCurrentState()
      this.srvGetAvailableTransitions()
      this.srvGetAvailableStates()
      // this.count++;
      // this.available_states.push(this.count)
      // console.log(this.count);
      // this.available_states.forEach(element => {
      //   console.log(element);
      // });
      // console.log(this.current_state);
    },
    dynobtn(par){
      console.log(par);
    },
    add_node(){
      if (this.input_node_name !== "") {
        this.added_nodes.push(this.input_node_name);
        console.log(this.added_nodes);
      } 
      
    },
    set_active_node(data){
      this.active_node = data;
      this.clear();
      this.update();
    },
    
    srvGetCurrentState() {
      // var ros = new ROSLIB.Ros({
      //   url : 'ws://localhost:9090'
      // });
      var lifecycleClient = new ROSLIB.Service({
        ros : this.ros,
        // name : `/lifecycle_joytovel/get_state`,
        name : `/${this.active_node}/get_state`,
        serviceType : 'lifecycle_msgs/GetState'
      });

      console.log('Calling Service');
      var request = new ROSLIB.ServiceRequest({});
      console.log('Result for service call on '
          + lifecycleClient.name
          + ': ');

      // var tempCurrentState = [];
      // lifecycleClient.callService(request, function(result) {
      //   console.log('Result for service call on '
      //     + lifecycleClient.name);
      //   console.log(result);
      //   console.log(result.current_state);
      //   tempCurrentState.push(result.current_state);
      // }, function(result) {
      //   console.log('Error');
      //   console.log(result);
      // });
      lifecycleClient.callService(request, (result) => {
        console.log(result);
        this.current_state = result.current_state;
      })
      // this.current_state = tempCurrentState;
      // console.log(this.current_state);
      // this.$forceUpdate(); 
      // this.update();
      // this.srvGetAvailableStates();

      // this.srvGetCurrentState()
      // this.srvGetAvailableTransitions()
      // this.srvGetAvailableStates()
    },

    
    srvGetAvailableStates() {
      // var ros = new ROSLIB.Ros({
      //   url : 'ws://localhost:9090'
      // });
      var lifecycleClient = new ROSLIB.Service({
        ros : this.ros,
        name : `/${this.active_node}/get_available_states`,
        // name : '/lifecycle_joytovel/get_available_states', 
        serviceType : 'lifecycle_msgs/GetAvailableStates'
      });

      console.log('Calling Service srvGetAvailableStates');
      var request = new ROSLIB.ServiceRequest({});

      // console.log('Result for service call on '
      //     + addTwoIntsClient.name
      //     + ': ');
      var tempArr = [];
      // lifecycleClient.callService(request, function(result) {
      //   console.log('Result for service call on '
      //     + lifecycleClient.name);
      //   console.log(result);
      //     console.log(result.available_states);
      //     result.available_states.forEach(element => {
      //       console.log(element);
      //       tempArr.push(element);
      //     });

      // }, function(result) {
      //   console.log('Error');
      //   console.log(result);
      // });

      lifecycleClient.callService(request, (result) => {
        console.log(result);
        result.available_states.forEach(element => {
            console.log(element);
            tempArr.push(element);
          });
        this.available_states = tempArr;
      })

      console.log("this.available_states");
      // this.available_states = tempArr;
      // console.log(this.available_states);
      // this.srvGetCurrentState()
      // this.srvGetAvailableTransitions()
      // this.srvGetAvailableStates()
    },

  srvGetAvailableTransitions() {
      // var ros = new ROSLIB.Ros({
      //   url : 'ws://localhost:9090'
      // });
      var lifecycleClient = new ROSLIB.Service({
        ros : this.ros,
        name : `/${this.active_node}/get_available_transitions`,
        // name : '/lifecycle_joytovel/get_available_transitions', 
        serviceType : 'lifecycle_msgs/GetAvailableTransitions'
      });

      console.log('Calling Service srvGetAvailableTransitions');
      var request = new ROSLIB.ServiceRequest({});

      // console.log('Result for service call on '
      //     + addTwoIntsClient.name
      //     + ': ');
      var tempArr = [];
      // lifecycleClient.callService(request, function(result) {
      //   console.log('Result for service call on '
      //     + lifecycleClient.name);
      //   console.log(result);
      //     console.log(result.available_transitions);
      //     result.available_transitions.forEach(element => {
      //       console.log(element);
      //       tempArr.push(element);
      //     });

      // }, function(result) {
      //   console.log('Error');
      //   console.log(result);
      // });

      lifecycleClient.callService(request, (result) => {
        console.log(result);
        result.available_transitions.forEach(element => {
            console.log(element);
            tempArr.push(element);
          });
        this.available_transitions = tempArr;
      }
        
      )


      console.log("this.available_transitions");
      // this.available_transitions = tempArr;
      console.log(this.available_transitions);
      // this.srvGetCurrentState()
      // // this.srvGetAvailableTransitions()
      // this.srvGetAvailableStates()
    },

    srvChangeState() {
      // var ros = new ROSLIB.Ros({
      //   url : 'ws://localhost:9090'
      // });
      var lifecycleClient = new ROSLIB.Service({
        ros : this.ros,
        name : `/${this.active_node}/change_state`,
        // name : '/lifecycle_joytovel/change_state', 
        serviceType : 'lifecycle_msgs/ChangeState'
      });

      console.log('Calling Service change_state');
  
      var r_id = parseInt(this.selected_transition);
      console.log(r_id );
      // r_id = 4;
      // var r_label = ""+tempSelected.label;
      var request = new ROSLIB.ServiceRequest({
        transition: {id: r_id, label : ""}
      });

      // console.log('Result for service call on '
      //     + addTwoIntsClient.name
      //     + ': ');

      lifecycleClient.callService(request, function(result) {
        console.log('Result for service call on '
          + lifecycleClient.name);
          console.log(request);
        console.log(result);
      }, function(result) {
        console.log('Error');
        console.log(result);
      });
      this.update();
    },

    subscribeToRosOut(){
      var listener = new ROSLIB.Topic({
          ros : this.ros,
          name : '/rosout',
          messageType : 'rcl_interfaces/msg/Log'
        });
        var msg = [];
        // listener.subscribe(function(message) {
        //   console.log('Received message on ' + listener.name + ': ' + message.level);
        //   console.log(message);
        //   msg.push(message.msg);
        //   console.log(this.active_node);
        //   // listener.unsubscribe();
        // });
        listener.subscribe((msg)=> {
          this.console_out.push(msg);
          if (this.console_out.length > 10) {
            this.console_out.shift();
          }
          console.log(msg);
          if (this.unsub) {
            listener.unsubscribe();
          }
        }) 
        // function(message) {
        //   console.log('Received message on ' + listener.name + ': ' + message.level);
        //   console.log(message);
        //   msg.push(message.msg);
        //   console.log(this.active_node);
        //   // listener.unsubscribe();
        // });
        console.log(msg);
    },
    mapLogLevel(level){
      if (level === 30) {
        return "WARN"
      } else {
          return "INFO"
      }

    }
  },

// computed: {
//       c_count: function () {
//       return this.count;
//     },
//       c_available_states: function () {
//       return this.available_states;
//     },
//       c_current_state: function () {
//         this.srvGetCurrentState();
//       return this.current_state;
//     },
//      c_available_transitions: function () {
//       return this.available_transitions;
//     },
//   },
};
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
.hello{
background: linear-gradient(#fffaaa, #fee8e5);
}
h3 {
  margin: 40px 0 0;
}
ul {
  list-style-type: none;
  padding: 0;
}
li {
  display: inline-block;
  margin: 0 10px;
}
a {
  color: #42b983;
}
</style>
