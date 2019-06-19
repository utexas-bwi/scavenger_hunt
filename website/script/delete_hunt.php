<?php

include_once 'connect.php';

$dbh = connect();
$dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
$results["error"] = true;

try {
  $hunt_id = $_POST["hunt_id"];
  $dbh->query("set foreign_key_checks=0;");
  $query = "delete from hunt_table where hunt_id=" . $hunt_id;
  $dbh->query($query);
  $dbh->query("set foreign_key_checks=1;");
  $results["error"] = false;
} catch (PDOException $e) {}

header("Content-type: application/json");
die(json_encode($results));

?>
