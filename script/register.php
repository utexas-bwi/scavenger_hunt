<?php
include 'connect.php';

$dbh = connect();

// Adds a hashed user ID into user_table if it isn't already present
try {
    $user_id = $_POST["user_id"];
    $user_email = $_POST["email"];
    $user_uni = $_POST["university"];
    $user_pass_hash = $_POST["pass_hash"];
    $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

    $query = "SELECT task_type, param_name FROM task_table";
	  $stmt = $dbh->query("select 1 from user_table where user_id=" . $user_id);
    $stmt->setFetchMode(PDO::FETCH_ASSOC);
    $user_exists = !!$stmt->fetch();

    if (!$user_exists)
      $dbh->query("insert into user_table values (" . $user_id . ", '" . $user_email . "', '" . $user_uni . "', '" . $user_pass_hash . "')");
} catch (PDOException $e) {
    die($e->getMessage());
}

$dbh = null;
?>
