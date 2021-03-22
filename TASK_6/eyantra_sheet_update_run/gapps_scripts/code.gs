function doGet(e){
  
  var ss = SpreadsheetApp.getActive();

  var sheet = ss.getSheetByName(e.parameter["id"]);

  var sheetid = sheet.getSheetId().toString(); // to get the sheet id of the sheet which is being updated and then convert it into a string value

  var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];

  var lastRow = sheet.getLastRow();

  var cell = sheet.getRange('a1');
  var col = 0;
  var d = new Date();

  for (i in headers){

    // loop through the headers and if a parameter name matches the header name insert the value

    if (headers[i] == "Timestamp")
    {
      var addedTime = Utilities.formatDate(d, "GMT+5:30","EEE MMM dd yyyy HH:mm:ss");// to change the timezone
      val = addedTime;
    }
    else
    {
      val = e.parameter[headers[i]]; 
    }
    // append data to the last row
    cell.offset(lastRow, col).setValue(val);
    col++;
  }
  // condition to check that if OrdersDispatched sheet is getting updated if true then do the following
  if (sheetid == "1478544825"){
   var k = sheet.getLastRow(); // to fetch data from the dispatch sheet
   var ordernum = sheet.getRange(k,4).getValue();
   var item = sheet.getRange(k,5).getValue();
   var qty = sheet.getRange(k,7).getValue();
   var city = sheet.getRange(k,8).getValue();
   var cost = sheet.getRange(k,9).getValue();
   var addedDate = sheet.getRange(k,11).getValue();
   var addedTime = Utilities.formatDate(addedDate, "GMT+5:30","dd-MM-yyyy HH:mm:ss"); //to change the time zone
   var dispatchdatetime = addedTime;
   var emailid = "eyrc.vb.0000@gmail.com";
   var subject = " Your Order is dispatched.";
   var message = "Hello"+"\n\n"+"Your Order has been dispatched, contact us if you have any questions.We are here to help you."+"\n\n"+"ORDER SUMMARY:"+"\n\n"+"Order Number: "+ordernum+"\n"+"Item: "+item+"\n"+"Quantity: "+qty+"\n"+"Dispatch Date and Time: "+dispatchdatetime+"\n"+"City: "+city+"\n"+"Cost: "+cost;
   MailApp.sendEmail(emailid,subject,message); // this function is used send to email alerts
  }

   // condition to check that if OrdersShipped sheet is getting updated if true then do the following
  else if(sheetid == "961576824"){
   var j = sheet.getLastRow();  // to fectch data from the shipped sheet
   var ordernums = sheet.getRange(j,4).getValue();
   var items = sheet.getRange(j,6).getValue();
   var qtys = sheet.getRange(j,8).getValue();
   var citys = sheet.getRange(j,5).getValue();
   var costs = sheet.getRange(j,9).getValue();
   var addedDate = sheet.getRange(j,11).getValue();
   var addedTime = Utilities.formatDate(addedDate, "GMT+5:30","dd-MM-yyyy HH:mm:ss"); // to chnage the timezone
   var shippeddatetime = addedTime;
   var estimate= sheet.getRange(j,12).getValue();
   var estimated = Utilities.formatDate(estimate, "GMT+5:30","dd-MM-yyyy");  
   var emailid = "eyrc.vb.0000@gmail.com"; 
   var subject = " Your Order is shipped.";
   var message = "Hello"+"\n\n"+"Your Order has been Shipped, contact us if you have any questions.We are here to help you."+"\n\n"+"ORDER SUMMARY:"+"\n\n"+"Order Number: "+ordernums+"\n"+"Item: "+items+"\n"+"Quantity: "+qtys+"\n"+"Shipped Date and Time: "+shippeddatetime+"\n"+"City: "+citys+"\n"+"Cost: "+costs+"\n"+"Estimated Time of Delivery: "+estimated;
   MailApp.sendEmail(emailid,subject,message); // this function is used send to email alerts
  }   
return ContentService.createTextOutput('success');
}





