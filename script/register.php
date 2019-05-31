<?php
include 'connect.php';

$dbh = connect();

try {
    $user_name = $_POST["user_name"];
    $user_id = $_POST["user_id"];
    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
    $query = "
    INSERT INTO `user_table` (" . $user_id . ", '" . $user_name . "', 'pwd')
    SELECT " . $user_id . " FROM `user_table` 
    WHERE NOT EXISTS (SELECT * FROM `table` 
      WHERE value1='stuff for value1' AND value2='stuff for value2') 
    LIMIT 1 
    ";
    $stmt = $dbh->query($query);
    $stmt = null; 
} catch (PDOException $e) {
    die($e->getMessage());
}

$dbh = null;
?>
