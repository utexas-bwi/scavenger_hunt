/**
 * Gets whether or not a user is logged in from firebase and passes that to
 * nav_bar.php so it can decide what tabs to populate it with
 */
function isLoggedIn() {
    $.ajax({
        data: 'loggedIn=' + (firebase.auth().currentNuser != null),
        url: 'nav_bar.php',
        method: 'POST',
        success: function(msg) {
            alert(msg);
        }
    });
}