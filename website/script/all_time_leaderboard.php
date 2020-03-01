<?php
  include_once 'connect.php';
  $dbh = connect();

  $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);

  include 'score_table.php';

  echo '<div class="leaderboard-body flex column grow">';

  // counts the number of universities that have partaken in a hunt
  $sort = "SELECT count(*) FROM score_table";
  $numRows = $dbh->query($sort)->fetchColumn();

  $sort = "SELECT * FROM score_table ORDER BY score DESC";
  $stmt = $dbh->query($sort);
  $stmt->setFetchMode(PDO::FETCH_ASSOC);
  include 'leaderboard.php';
?>