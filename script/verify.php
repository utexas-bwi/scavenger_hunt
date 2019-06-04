<?php
  include 'connect.php';
  $dbh = connect();
  
  $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

  // obtains the files that have not yet been verified by a user
  $query = "SELECT * FROM proof_table WHERE verified = 0";
  $stmt = $dbh->query($query);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);

  // goes through the filenames of proofs that have not been verified and displays on page
  while($item = $stmt -> fetch()){
    $imageFilename = $item['filename'];
    $proofId = $item['proof_id'];
    include 'components/image-verify.html';
  }
?>
