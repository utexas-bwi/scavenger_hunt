firebase.auth().onAuthStateChanged(function(user) {
  if (user) {
    // User is signed in.

    // document.getElementById("user_div").style.display = "block";
    // document.getElementById("login_div").style.display = "none";

    // document.getElementById('navbar').innerHTML = 'components/navBar.html';
    $("#navbar").load("components/navBar.html");

    console.log("im in");

  } else {
    // No user is signed in.

    // document.getElementById("user_div").style.display = "none";
    // document.getElementById("login_div").style.display = "block";

    $("#navbar").load("components/navBar_noauth.html");

    console.log("no user :(");

  }
});

function login(){

  var userEmail = document.getElementById("email_field").value;
  var userPass = document.getElementById("password_field").value;

  firebase.auth().signInWithEmailAndPassword(userEmail, userPass).catch(function(error) {
    // Handle Errors here.
    var errorCode = error.code;
    var errorMessage = error.message;

    window.alert("Error : " + errorMessage);

    // ...
  });
}

function logout(){
  firebase.auth().signOut();
  window.location = "login.html";
}
