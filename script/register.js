/**
 * Register a new user with Firebase
 */
function register() {
  var name = document.getElementById('name').value;
  var uni = document.getElementById('uni').value;
  var password = document.getElementById('pass').value;
  var email = document.getElementById('email').value;
  var re_pass = document.getElementById('re_pass').value;

  // Check input lengths
  if (name.length == 0) {
      alert("You must enter a name!");
      return;
  }

  if (uni.length == 0) {
    alert("You must enter a university!");
    return;
  }

  if (password.length < 8) {
      alert('Password must be at least 8 characters long.');
      return;
  }

  // I don't know what this is doing
  for (var i = 0; i < password.length; i++) {
    if (password[i].type == 'password')
        password[i].setAttribute('type','text');
  }

  for (var i = 0; i < re_pass.length; i++) {
    if (re_pass[i].type == 'password')
      re_pass[i].setAttribute('type','text');
  }

  // Check that passwords match
  if (password != re_pass) {
      alert('Passwords do not match. Try again.');
      return;
  }

  window.university = uni;

  // Register with Firebase
  var errorMessage = "a";
  firebase.auth().createUserWithEmailAndPassword(email, password).catch(function(error) {
    var errorCode = error.code;
    errorMessage = error.message;
    console.log(errorMessage);
    alert('Something went wrong. Let us know and try again later.');
  });
}