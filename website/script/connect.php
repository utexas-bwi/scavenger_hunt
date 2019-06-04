<?php
    function connect() {
        $username = 'bwi';
        $password = 'segbot3768';
        return new PDO('mysql:host = localhost:3306; dbname=scavenger_hunt', $username, $password);
    }
?>
