function addRow() {
    const table = document.getElementById("task-table");
    const row = document.getElementById("tr-hide");
    row.classList.remove("hide");
    row.classList.remove("table-line");
    table.appendChild(row);
}

function removeRow(cell) {
    document.getElementById("task-table").deleteRow(cell.parentNode.parentNode.rowIndex);
}

function saveChanges() {
    const table = document.getElementById("task-table");
    // TODO: export data to SQL TaskType database 
}
