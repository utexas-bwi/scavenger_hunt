<?php
if ($_POST['save_tasks']) {
    $newrows = $_POST['task_table'];
    // Create connection
    include 'connect.php';
    $dbh = connect();
    try {
        // get copy of old database
        $dbh->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
        $stmt = $dbh->query("SELECT * FROM task_table");
        $stmt->setFetchMode(PDO::FETCH_ASSOC);
        $oldrows = [];
        while($oldrow = $stmt->fetch()) {
            array_push($oldrows, $oldrow);
        }
        // update rows
        for ($odx = 0; $odx < count($oldrows); ++$odx) {
            for ($rdx = 0; $rdx < count($newrows); ++$rdx) {
                if ($newrows[$rdx][0] == $oldrows[$odx]['id']) {
                    echo $rdx. " has same id as " . $odx . "\n";
                    // fix ordering of values
                    $id = $newrows[$rdx][0];
                    array_splice($newrows[$rdx], 0, 1);
                    array_push($newrows[$rdx], $id);
                    // update row
                    $sql = "UPDATE task_table SET task_type=?, param_name=?, proof_type=?, score=?, description=? where id=?";
                    $dbh->prepare($sql)->execute($newrows[$rdx]);
                    // remove rows we changed
                    array_splice($oldrows, $odx, 1);
                    array_splice($newrows, $rdx, 1);
                    $odx = -1;
                    $rdx = -1;
                }
            }
        }
        // add/remove any leftover rows
        if (!empty($oldrows)) {
            // remove all oldrows
            for ($odx = 0; $odx < count($oldrows); ++$odx) {
                echo "removing " . $oldrows[$odx]['id'] . "\n";
                $sql = "DELETE from task_table WHERE id=" . $oldrows[$odx]['id'];
                $dbh->prepare($sql)->execute();
            }
        }
        if (!empty($newrows)) {
            // add all newrows
            echo "add ". count($newrows). " rows\n";
            for ($rdx = 0; $rdx < count($newrows); ++$rdx) {
                $sql = "INSERT into task_table VALUES (?, ?, ?, ?, ?, ?)";
                $dbh->prepare($sql)->execute($newrows[$rdx]);
            }
        }
    } catch (PDOException $e) {
        die($e->getMessage());
    }
    // Close connection
    $dbh = null;
}
?>
