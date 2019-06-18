// Point nav bar items into public_html folder if on index
function indexNav(page) {
  var path = window.location.pathname;
  var page = path.split("/").pop();

  if (page == "index.html" || page == "") {
    var e = document.getElementById("navbar-rules");
    if (e) e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
    e = document.getElementById("navbar-task");
    if (e) e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
    e = document.getElementById("navbar-leaderboard");
    if (e) e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
    e = document.getElementById("navbar-contact");
    if (e) e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
    e = document.getElementById("navbar-userhunts");
    if (e) e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
    e = document.getElementById("navbar-register");
    if (e) e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
    e = document.getElementById("navbar-login");
    if (e) e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
    e = document.getElementById("navbar-verify");
    if (e) e.setAttribute('href', 'public_html/' + e.getAttribute('href'));
  }
}

// Page updates when the authentication state changes
firebase.auth().onAuthStateChanged(function(user) {
  var path = window.location.pathname;
  var page = path.split("/").pop();
  var verified = user && user.emailVerified;
  var verifyRestrictedPages = [
    "userhunts.html",
    "task.html",
    "verify.html"
  ];
  var logoutRestrictedPages = [
    "login.html"
  ];

  // Force redirect if user is logged out and on a restricted page
  if (!verified && verifyRestrictedPages.includes(page))
    window.location = "../index.html";

  // Force redirect if user is logged in and on a restricted page
  if (user && logoutRestrictedPages.includes(page))
    window.location = "../index.html";

  // Nav bar changes based on user verification
  if (!user)
    $("#navbar").load(htmlComponentsPath + "navBarNoAuth.html", indexNav);
  else if(!verified)
    $("#navbar").load(htmlComponentsPath + "navBarUnverified.html", indexNav);
  else
    $("#navbar").load(htmlComponentsPath + "navBarAuth.html", indexNav);

  if (user && !verified && page == "register.html") {
    // Send verification email if necessary
    user.sendEmailVerification().then(function() {
      const message = document.getElementById("alert-msg");
      message.classList.remove("hidden-msg");
      message.classList.remove("alert-danger");
      message.classList.add("alert-success");
      message.textContent = "A verification email has been sent to " + user.email + ".";
      setTimeout(function() {
          message.classList.add("hidden-msg");
          message.classList.remove("alert-success");
          window.location = "../index.html";
      }, 2000);
    }).catch(function(error) {
      console.log("Failed to send verification email!");
    });

    // Attempt to add the user to the SQL database if they haven't been already
    $.ajax({
      type: "POST",
      url: "../../script/register.php",
      data: {
        "user_id": user.uid.hashCode(),
        "email": user.email,
        "university": window.university,
        "pass_hash": document.getElementById("pass").value.hashCode()
      }
    });
  }

  if (verified && page == "login.html")
    window.location = "userhunts.html";
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
