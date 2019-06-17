// Page updates when the authentication state changes
firebase.auth().onAuthStateChanged(function(user) {
  /*setTimeout(function() {
    const message = document.getElementById("alert-msg");
    message.classList.remove("hidden-msg");
    message.classList.remove("alert-danger");
    message.classList.add("alert-success");
    message.textContent = "A verification email has been sent to haha yes.";
    setTimeout(function() {
        message.classList.add("hidden-msg");
        message.classList.remove("alert-success");
    }, 4000);
  }, 500);*/

  var path = window.location.pathname;
  var page = path.split("/").pop();
  var verified = !!user && firebase.auth().currentUser.emailVerified;
  var authRestrictedPages = [
    "userhunts.html",
    "task.html"
  ];

  // Nav bar changes based on user verification
  if (!user)
    $("#navbar").load(htmlComponentsPath + "navBarNoAuth.html");
  else if(!verified)
    $("#navbar").load(htmlComponentsPath + "navBarUnverified.html");
  else
    $("#navbar").load(htmlComponentsPath + "navBarAuth.html");

  // Send verification email if necessary
  if (!!user && !verified && page == "register.html") {
    user.sendEmailVerification().then(function() {
      const message = document.getElementById("alert-msg");
      message.classList.remove("hidden-msg");
      message.classList.remove("alert-danger");
      message.classList.add("alert-success");
      message.textContent = "A verification email has been sent to " + user.email + ".";
      setTimeout(function() {
          message.classList.add("hidden-msg");
          message.classList.remove("alert-success");
      }, 4000);
    }).catch(function(error) {
      console.log("Failed to send verification email!");
    });
  }

  // Bounce the user if they don't qualify to be on this page
  if (authRestrictedPages.includes(page) && (!user || !verified))
    window.location = "../index.html";

  // User just signed in
  if (user) {
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
          if (page == "register.html" && verified)
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
      e = document.getElementById("navbar-leaderboard");
      e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
      e = document.getElementById("navbar-contact");
      e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
      if(verified){
        e = document.getElementById("navbar-task");
        e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
        e = document.getElementById("navbar-userhunts");
        e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
      }
    };

    // Graceful redirect to userhunts page upon login
    if (page == "login.html")
      window.location = "userhunts.html";

    console.log("im in");
  // User just signed out
  } else {
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
