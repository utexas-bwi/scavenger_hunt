/**
 * Sends the firebase user's ID thru post to some php script
 * @param {url of recipient php script} url
 * @param {divider id to inject response html into} div
 */
function sendUserId(url, div) {
  firebase.auth().onAuthStateChanged(function(user) {
    if (user) {
      $.ajax({
        type: "POST",
        url,
        data: {"user_id": user.uid.hashCode()},
        success: function(output) {
          $(document).ready(function () {
            $('#' + div).html(output);
          });
          console.log("yep that worked");
        },
        error: function(request, status, error){
          alert("Error: " + error);
          console.log("no sir");
        }
      });
    }
  });
}