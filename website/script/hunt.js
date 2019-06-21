function viewHunt(id) {
    window.location.href = "createhunt.html?type=edit_hunt&id=" + id;
}

function newHunt() {
    window.location.href = "createhunt.html?type=create_hunt";
}

function deleteHunt() {
  if (confirm("Are you sure you want to delete this hunt?")) {
    if (window.location.href.indexOf("id") > -1) {
        const vars = {};
        const parts = window.location.href.replace(/[?&]+([^=&]+)=([^&]*)/gi, function(m,key,value) {
            vars[key] = value;
        });
        var hunt_id = vars["id"];

        $.ajax({
            type: "POST",
            url: "../script/delete_hunt.php",
            data: {
                hunt_id: hunt_id,
            },
            success: function(data) {
              if (!data.error) {
                window.location = "userhunts.html";
              } else {
                const message = document.getElementById("save-msg");
                message.classList.remove("hidden-msg");
                message.classList.remove("alert-success");
                message.classList.add("alert-danger");
                message.textContent = "Failed to delete hunt. Something broke!";
                setTimeout(function() {
                    message.classList.add("hidden-msg");
                    message.classList.remove("alert-danger");
                }, 10000);
              }
            },
            failure: function() {
              const message = document.getElementById("save-msg");
              message.classList.remove("hidden-msg");
              message.classList.remove("alert-success");
              message.classList.add("alert-danger");
              message.textContent = "Failed to delete hunt. Something really, really broke!";
              setTimeout(function() {
                  message.classList.add("hidden-msg");
                  message.classList.remove("alert-danger");
              }, 10000);
            }
        });
    }
  }
}

function submitHunt() {
    // extract data from table
    const table = document.getElementById("hunt-table");
    const data = [];
    for (let rdx = 3; rdx < table.rows.length; ++rdx) {
        const rowdata = [];
        const row = table.rows[rdx];
        // skip last two cells but iterate over rest bc last 2 are buttons
        for (let cdx = 0; cdx < row.cells.length - 2; ++cdx) {
            const cell = row.cells[cdx];
            rowdata.push(cell.textContent);
        }
        console.log(rowdata);
        data.push(rowdata);
    }
    // get hunt ID of hunt
    let id = -1;
    if (window.location.href.indexOf("id") > -1) {
        const vars = {};
        const parts = window.location.href.replace(/[?&]+([^=&]+)=([^&]*)/gi, function(m,key,value) {
            vars[key] = value;
        });
        id = vars["id"];
    }
    // get name of hunt from URL or from page if new hunt
    const new_name = document.getElementById("hunt-name").textContent.trim();

    // get start and end date
    // if no expiration, send ""
    var release = new Date(document.getElementById("release_date").value);
    var end = new Date(document.getElementById("end_date").value);

    if(document.getElementById("no_expiration").checked)
      end = "";

    // send data to php function
    const url = '../script/save_hunt_table.php';
    var hash = firebase.auth().currentUser.uid.hashCode();
    $.ajax({
        type: "POST",
        url,
        data: {
            save_hunt: true,
            hunt_table: data,
            hunt_id:id,
            hunt_name: new_name,
            user_id: hash,
            release_date: release,
            end_date: end
        },
        success: function(data) {
            if (!data.error) {
                const message = document.getElementById("save-msg");
                message.classList.remove("hidden-msg");
                message.classList.remove("alert-danger");
                message.classList.add("alert-success");
                message.textContent = "Your hunt has been saved.";
                setTimeout(function() {
                    message.classList.add("hidden-msg");
                    message.classList.remove("alert-success");
                }, 2000);
            } else {
                const message = document.getElementById("save-msg");
                message.classList.remove("hidden-msg");
                message.classList.remove("alert-success");
                message.classList.add("alert-danger");
                message.textContent = data.data;
                setTimeout(function() {
                    message.classList.add("hidden-msg");
                    message.classList.remove("alert-danger");
                }, 10000);
            }
        }
    });
}
