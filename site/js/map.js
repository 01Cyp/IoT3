function sleep(seconds){
    var waitUntil = new Date().getTime() + seconds*1000;
    while(new Date().getTime() < waitUntil) true;
}

var mymap = L.map('mapid').setView([50.607806, 3.136296], 17);

L.tileLayer('http://{s}.tile.osm.org/{z}/{x}/{y}.png', {}).addTo(mymap);

function addMarker() {
    var marker = L.marker([50.607806, 3.136296]).addTo(mymap);

    marker.bindPopup("Your Gear").openPopup();
}
