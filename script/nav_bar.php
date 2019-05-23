<?php
/**
 * Builds the nav bar based on user auth
 */
if (!$_POST['loggedIn'])
    include 'components/navBar_noauth.html';
else
    include 'components/navBar.html';
?>
