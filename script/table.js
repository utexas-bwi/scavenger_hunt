
/* function to make cell editable */

function editable(span) {
    const tr = span.parentNode.parentNode;
    console.log(tr);
    if (span.classList.contains("glyphicon-pencil")) {
        for (let i = 1; i < tr.childNodes.length - 4; i = i + 2) {
            tr.childNodes[i].setAttribute("contentEditable","true");
            tr.childNodes[i].classList.add("editable");
        }
        span.classList.remove("glyphicon-pencil");
        span.classList.add("glyphicon-ok");
    } else {
        for (let i = 1; i < tr.childNodes.length - 4; i = i + 2) {
            tr.childNodes[i].setAttribute("contentEditable","false");
            tr.childNodes[i].classList.remove("editable");
        }
        span.classList.add("glyphicon-pencil");
        span.classList.remove("glyphicon-ok");
    }
}


/* functions to add to task table */

function addRow(cell) {
    const table = cell.parentNode.parentNode.parentNode;
    const row = document.getElementById("tr-hide").cloneNode(true);
    row.classList.remove("hide");
    row.classList.remove("table-line");
    table.appendChild(row);
}

function removeRow(cell) {
    const table = cell.parentNode.parentNode.parentNode;
    table.deleteRow(cell.parentNode.parentNode.rowIndex);
}

function saveChanges() {
    const table = document.getElementById("task-table");
    // TODO: export data to SQL TaskType database 
}

/* functions to add to hunt table */

function addTask(task) {
    const table = document.getElementById("hunt-table");
    const row = document.getElementById("tr-hide").cloneNode(true);
    row.childNodes[1].textContent = task;
    row.classList.remove("hide");
    row.classList.remove("table-line");
    table.appendChild(row);
}
