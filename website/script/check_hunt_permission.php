<?php

include_once 'connect.php';

$dbh = connect();
$dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
$results["error"] = true;
$results["no_perm"] = true;

try {
  $hunt_id = $_POST["hunt_id"];
  $user_id = $_POST["user_id"];
  $query = "select * from hunt_table where hunt_id=" . $hunt_id;
  $stmt = $dbh->query($query);
  $result = $stmt->fetch();

  if ($result) {
    $results["no_perm"] = $result["user_id"] != $user_id;
  }

  $results["error"] = false;

} catch (PDOException $e) {}

header("Content-type: application/json");
die(json_encode($results));

?>
