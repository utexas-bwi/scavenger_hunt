// Page updates when the authentication state changes
firebase.auth().onAuthStateChanged(function(user) {
  var path = window.location.pathname;
  var page = path.split("/").pop();
  var authRestrictedPages = [
    "userhunts.html",
    "task.html"
  ];

  // User just signed in
  if (user) {
    $("#navbar").load(htmlComponentsPath + "navBarAuth.html");

    // Attempt to add the user to the SQL database if they haven't been already
    if (page == "register.html") {
      $.ajax({
        type: "POST",
        url: "../../script/register.php",
        data: {
          "user_id": user.uid.hashCode(),
          "email": user.email,
          "university": window.university,
          "pass_hash": document.getElementById("pass").value.hashCode()
        },
        success: function(data) {
          if (page == "register.html")
            window.location = "userhunts.html";
        },
        failure: function(data) {
          console.log("Something went wrong with new user reg");
        }
      });
    }

    // Point each of the nav bar items into the public_html folder (necessary
    // for navigation from index page to work)
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

    // Graceful redirect to userhunts page upon login
    if (page == "login.html")
      window.location = "userhunts.html";

    console.log("im in");
  // User just signed out
  } else {
    // Bounce the logged-out user if they're on a page that requires auth
    if (authRestrictedPages.includes(page))
      window.location = "../index.html";

    $("#navbar").load(htmlComponentsPath + "navBarNoAuth.html");

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

    console.log("no user :(");
  }
});

// Called when clicking login on login page
function login(){
  var userEmail = document.getElementById("email_field").value;
  var userPass = document.getElementById("password_field").value;

  firebase.auth().signInWithEmailAndPassword(userEmail, userPass).catch(function(error) {
    var errorCode = error.code;
    var errorMessage = error.message;
    window.alert("Error : " + errorMessage);
  });
}

// Called when clicking logout in nav bar
function logout(){
  firebase.auth().signOut();
  window.location = "index.html";
}
