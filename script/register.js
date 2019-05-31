/**
 * Register a new user with Firebase
 */
function register() {
  var password = document.getElementById('pass').value;
  var email = document.getElementById('email').value;
  var re_pass = document.getElementById('re_pass').value;
  if (password.length < 8) {
      alert('Password must be at least 8 characters long');
      return;
  }
  for (var i = 0; i < password.length; i++) {
  if (password[i].type == 'password')
      password[i].setAttribute('type','text');
  }
  for (var i = 0; i < re_pass.length; i++) {
  if (re_pass[i].type == 'password')
      re_pass[i].setAttribute('type','text');
  }
  if (password != re_pass) {
      alert('Password and verification password must be equal');
      return;
  }
  console.log(document.getElementById('email').value);
  var errorMessage = "a";
  firebase.auth().createUserWithEmailAndPassword(email, password).catch(function(error) {
    var errorCode = error.code;
    errorMessage = error.message;
    console.log(errorMessage);
  });
}