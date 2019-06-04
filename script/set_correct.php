<?php
  include 'connect.php';
  $dbh = connect();
  
  $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

  // sets correct to be true
  $sql = "UPDATE proof_table SET correct = 1 WHERE proof_id = ?";
  $stmt = $dbh->prepare($sql);
  $stmt->execute([$_POST['proofId']]);
?>
