firebase.auth().onAuthStateChanged(function(user) {
  var path = window.location.pathname;
  var page = path.split("/").pop();

  // User is signed in
  if (user) {
    // document.getElementById("user_div").style.display = "block";
    // document.getElementById("login_div").style.display = "none";

    // document.getElementById('navbar').innerHTML = 'components/navBar.html';
    $("#navbar").load(htmlComponentsPath + "navBarAuth.html");

    console.log("im in");

    window.onload = function() {
      var e = document.getElementById("navbar-rules");
      e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
      e = document.getElementById("navbar-task");
      e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
      e = document.getElementById("navbar-leaderboard");
      e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
      e = document.getElementById("navbar-contact");
      e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
      e = document.getElementById("navbar-userhunts");
      e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
    };
  // No user is signed in
  } else {
    // document.getElementById("user_div").style.display = "none";
    // document.getElementById("login_div").style.display = "block";

    $("#navbar").load(htmlComponentsPath + "navBarNoAuth.html");

    console.log("no user :(");

    // Points each of the navbar items into the public_html folder
    window.onload = function() {
      var e = document.getElementById("navbar-rules");
      e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
      e = document.getElementById("navbar-leaderboard");
      e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
      e = document.getElementById("navbar-contact");
      e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
      e = document.getElementById("navbar-register");
      e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
      e = document.getElementById("navbar-login");
      e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
    };
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
