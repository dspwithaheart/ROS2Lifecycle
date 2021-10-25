<template>
  <!-- <MDBBtn color="primary">Primary</MDBBtn> -->
  <!-- <img alt="Vue logo" src="./assets/logo.png"> -->
  <HelloWorld msg="Lifecycle Node Control Dashboard"/>
  <!-- <button type="button" class="btn btn-secondary me-2" 
  data-bs-container="body" data-bs-placement="top" 
  data-bs-content="Top popover">      Popover on top
    </button> -->
</template>

<script>
import HelloWorld from './components/HelloWorld.vue'
import "bootstrap/dist/css/bootstrap.min.css";
import "bootstrap/dist/js/bootstrap.min.js";

export default {
  name: 'App',
  components: {
    HelloWorld,
  }
}
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}
</style>
