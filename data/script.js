var logScrollPaused = false;
var logLines = [];
var MAX_LOG_LINES = 100000;

function xget(url,cb){var x=new XMLHttpRequest();x.open("GET",url,true);if(cb)x.onreadystatechange=function(){if(x.readyState==4)cb(x)};x.send()}
function toggle(id,onUrl,offUrl){var e=document.getElementById(id);if(!e)return;var c=e.checked;xget(c?onUrl:offUrl)}
function sync(id,val){var e=document.getElementById(id);if(e)e.checked=(val==="ON")}
function s(id,v){var e=document.getElementById(id);if(e)e.innerHTML=v}
function confirmPost(msg,action){if(confirm(msg)){var f=document.createElement('form');f.method='POST';f.action=action;document.body.appendChild(f);f.submit()}}

// Intercept form submissions to config endpoints — submit via XHR and refresh config
var configEndpoints = ['/setAdvancedParams', '/setStellarium', '/setWindThresholds',
  '/setWeatherLocation', '/setWeatherApiKey', '/setAngleOffsets',
  '/setDualMotorMaxSpeed', '/setSingleMotorMaxSpeed', '/setPassword'];
document.addEventListener('submit', function(e) {
  var form = e.target;
  var action = form.getAttribute('action') || '';
  var isConfig = false;
  for (var i = 0; i < configEndpoints.length; i++) {
    if (action === configEndpoints[i]) { isConfig = true; break; }
  }
  if (!isConfig) return;
  e.preventDefault();
  var params = [];
  for (var i = 0; i < form.elements.length; i++) {
    var el = form.elements[i];
    if (!el.name || el.type === 'submit' || el.type === 'button') continue;
    if ((el.type === 'checkbox' || el.type === 'radio') && !el.checked) continue;
    if (el.disabled) continue;
    params.push(encodeURIComponent(el.name) + '=' + encodeURIComponent(el.value));
  }
  var xhr = new XMLHttpRequest();
  xhr.open(form.method || 'POST', action, true);
  xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');
  xhr.onreadystatechange = function() {
    if (xhr.readyState == 4) {
      if (xhr.status >= 200 && xhr.status < 300) {
        fetchConfig();
      } else if (xhr.status >= 400) {
        alert('Error: ' + (xhr.responseText || 'Request failed'));
      }
    }
  };
  xhr.send(params.join('&'));
});

// Fetch configuration data on page load (and after settings changes)
// Retries on failure since the ESP32 may be busy serving other page assets
function fetchConfig() {
  xget("/config", function(x) {
    if (x.status != 200) { setTimeout(fetchConfig, 2000); return; }
    var data;
    try { data = JSON.parse(x.responseText); } catch(e) { setTimeout(fetchConfig, 2000); return; }

    s("http_port",data.http_port);
    s("rotctl_port",data.rotctl_port);
    s("hostname",data.hostname);
    s("wifissid",data.wifissid);
    s("loginUser",data.loginUser);
    s("passwordStatus",data.passwordStatus);
    s("maxDualMotorAzSpeed",data.maxDualMotorAzSpeed);
    s("maxDualMotorElSpeed",data.maxDualMotorElSpeed);
    s("maxSingleMotorAzSpeed",data.maxSingleMotorAzSpeed);
    s("maxSingleMotorElSpeed",data.maxSingleMotorElSpeed);
    s("stellariumPollingOn",data.stellariumPollingOn);
    s("stellariumServerIPText",data.stellariumServerIPText);
    s("stellariumServerPortText",data.stellariumServerPortText);
    s("toleranceAz",data.toleranceAz);
    s("toleranceEl",data.toleranceEl);
    s("P_el",data.P_el);
    s("P_az",data.P_az);
    s("MIN_EL_SPEED",data.MIN_EL_SPEED);
    s("MIN_AZ_SPEED",data.MIN_AZ_SPEED);
    s("MIN_AZ_TOLERANCE",data.MIN_AZ_TOLERANCE);
    s("MIN_EL_TOLERANCE",data.MIN_EL_TOLERANCE);
    s("MAX_FAULT_POWER_AZ",data.MAX_FAULT_POWER_AZ);
    s("MAX_FAULT_POWER_EL",data.MAX_FAULT_POWER_EL);
    s("MAX_FAULT_POWER_TOTAL",data.MAX_FAULT_POWER_TOTAL);
    s("MIN_VOLTAGE_THRESHOLD",data.MIN_VOLTAGE_THRESHOLD);
    s("I_el",data.I_el);
    s("I_az",data.I_az);
    s("D_el",data.D_el);
    s("D_az",data.D_az);
    s("azOffset",data.azOffset);
    s("elOffset",data.elOffset);
    s("weatherEnabled",data.weatherEnabled);
    s("weatherApiKeyConfigured",data.weatherApiKeyConfigured);
    s("weatherLocationConfigured",data.weatherLocationConfigured);
    s("weatherLatitude",data.weatherLatitude);
    s("weatherLongitude",data.weatherLongitude);
    s("windSafetyEnabled",data.windSafetyEnabled);
    s("windBasedHomeEnabled",data.windBasedHomeEnabled);
    s("windSpeedThreshold",data.windSpeedThreshold);
    s("windGustThreshold",data.windGustThreshold);
    s("autoHomeEnabled",data.autoHomeEnabled);
    s("autoHomeMins",data.autoHomeMins);
    s("smoothTrackingEnabled",data.smoothTrackingEnabled);
    s("smKalQ",data.smKalQ);
    s("smKalR",data.smKalR);
    s("smMinAzSpd",data.smMinAzSpd);
    s("smMinElSpd",data.smMinElSpd);

    sync("weatherPolling",data.weatherEnabled);
    sync("windSafetyToggle",data.windSafetyEnabled);
    sync("windBasedHomeToggle",data.windBasedHomeEnabled);
    sync("autoHomeToggle",data.autoHomeEnabled);
    sync("smoothTrackingToggle",data.smoothTrackingEnabled);
    sync("stellariumOn",data.stellariumPollingOn);
  });
}
fetchConfig();

// Real-time status polling
var pollInFlight = false;
setInterval(function() {
  if (pollInFlight) return;
  pollInFlight = true;
  var xhr = new XMLHttpRequest();
  xhr.timeout = 5000;
  xhr.open("GET", "/status", true);
  xhr.onreadystatechange = function() {
    if (xhr.readyState == 4) {
      pollInFlight = false;
      if (xhr.status != 200) return;
      var data;
      try {
        data = JSON.parse(xhr.responseText);
      } catch(e) {
        return;
      }
      s("correctedAngle_el",data.correctedAngle_el);
      s("correctedAngle_az",data.correctedAngle_az);
      s("setpoint_az",data.setpoint_az);
      s("setpoint_el",data.setpoint_el);
      s("setPointState_az",data.setPointState_az);
      s("setPointState_el",data.setPointState_el);
      s("error_az",data.error_az);
      s("error_el",data.error_el);
      s("el_startAngle",data.el_startAngle);
      s("needs_unwind",data.needs_unwind);
      s("calMode",data.calMode);
      sync("toggleCal",data.calMode);
      sync("singleMotorMode",data.singleMotorModeText);
      s("i2cErrorFlag_az",data.i2cErrorFlag_az);
      s("i2cErrorFlag_el",data.i2cErrorFlag_el);
      s("badAngleFlag",data.badAngleFlag);
      s("magnetFault",data.magnetFault);
      s("faultTripped",data.faultTripped);
      s("serialActive",data.serialActive);
      s("singleMotorModeText",data.singleMotorModeText);
      s("isAzMotorLatched",data.isAzMotorLatched);
      s("isElMotorLatched",data.isElMotorLatched);
      s("motorSpeedPctAz",data.motorSpeedPctAz);
      s("motorSpeedPctEl",data.motorSpeedPctEl);
      s("stellariumConnActive",data.stellariumConnActive);
      s("inputVoltage",data.inputVoltage);
      s("currentDraw",data.currentDraw);
      s("rotatorPowerDraw",data.rotatorPowerDraw);
      s("ip_addr",data.ip_addr);
      s("rotctl_client_ip",data.rotctl_client_ip);
      s("bssid",data.bssid);
      s("wifi_channel",data.wifi_channel);

      s("rssi",data.rssi);
      var level = data.level;
      for(var i=1;i<5;i++)document.getElementById('bar'+i).classList.toggle('active',level>=i);

      s("windStowActive",data.windStowActive);
      s("windStowReason",data.windStowReason);
      s("windTrackingActive",data.windTrackingActive);
      s("windTrackingStatus",data.windTrackingStatus);
      s("emergencyStowActive",data.emergencyStowActive);
      s("stowDirection",data.stowDirection);

      s("directionLockStatus",data.directionLockEnabled);
      s("smoothTrackingActive",data.smoothTrackingActive);
      var dlToggle=document.getElementById("disableDirectionLockToggle");
      if(dlToggle)dlToggle.checked=(data.directionLockEnabled==="OFF");

      sync("extendedElToggle",data.extendedElEnabled);
      var elSetpointLabel = document.getElementById("elSetpointLabel");
      if (elSetpointLabel) {
        elSetpointLabel.innerHTML = (data.extendedElEnabled === "ON") ? "Set Elevation (-90 - 90):" : "Set Elevation (0 - 90):";
      }

      var windStowAlert = document.getElementById("windStowAlert");
      var windStowMessage = document.getElementById("windStowMessage");
      if (data.emergencyStowActive === "YES") {
        windStowMessage.innerHTML = "⚠️ EMERGENCY WIND STOW ACTIVE: " + data.windStowReason;
        windStowAlert.style.display = "block";
      } else {
        windStowAlert.style.display = "none";
      }

      if (data.newLogMessages && data.newLogMessages.trim() !== "") {
        var newLines = data.newLogMessages.split('\n');
        for (var i = 0; i < newLines.length; i++) {
          if (newLines[i].trim() !== "") {
            logLines.push(newLines[i]);
            if (logLines.length > MAX_LOG_LINES) {
              logLines.shift();
            }
          }
        }
      }

      updateLogDisplay();

      if (document.getElementById("currentDebugLevel")) {
        s("currentDebugLevel",data.currentDebugLevel);
        updateDebugLevelDropdown(data.currentDebugLevel);
      }

      if (document.getElementById("serialOutputDisabled")) {
        s("serialOutputDisabled",data.serialOutputDisabled ? "True" : "False");
      }

      var disableSerialOutputCheckbox = document.getElementById("disableSerialOutput");
      if (disableSerialOutputCheckbox) {
        disableSerialOutputCheckbox.checked = data.serialOutputDisabled;
      }

      var azimuth = data.correctedAngle_az;
      var elevation = data.correctedAngle_el;
      var setpoint_az = data.setpoint_az;
      var setpoint_el = data.setpoint_el;
      var kalmanAz = data.kalmanAzPos != null ? parseFloat(data.kalmanAzPos) : null;
      var kalmanEl = data.kalmanElPos != null ? parseFloat(data.kalmanElPos) : null;
      if (data.kalmanAzVel) s("kalmanAzVel", data.kalmanAzVel + " deg/s");
      if (data.kalmanElVel) s("kalmanElVel", data.kalmanElVel + " deg/s");

      s("weatherDataValid",data.weatherDataValid);
      s("weatherLastUpdate",data.weatherLastUpdate);

      s("currentWindSpeed",data.currentWindSpeed);
      s("currentWindDirection",formatWindDirection(data.currentWindDirection));
      s("currentWindGust",data.currentWindGust);
      s("currentWeatherTime",formatWeatherTime(data.currentWeatherTime));

      var hasForecastData = data.forecastWindSpeed &&
                            data.forecastWindDirection &&
                            data.forecastWindGust &&
                            data.forecastTimes;

      if (hasForecastData) {
        for (var i = 0; i < 3; i++) {
          var time = (data.forecastTimes.length > i) ? data.forecastTimes[i] : null;
          var speed = (data.forecastWindSpeed.length > i) ? data.forecastWindSpeed[i] : null;
          var direction = (data.forecastWindDirection.length > i) ? data.forecastWindDirection[i] : null;
          var gust = (data.forecastWindGust.length > i) ? data.forecastWindGust[i] : null;

          s("forecastTime" + i,formatWeatherTime(time));
          s("forecastWindSpeed" + i,(speed !== null) ? speed : "N/A");
          s("forecastWindDirection" + i,formatWindDirection(direction));
          s("forecastWindGust" + i,(gust !== null) ? gust : "N/A");
        }
      } else {
        for (var i = 0; i < 3; i++) {
          s("forecastTime" + i,"N/A");
          s("forecastWindSpeed" + i,"N/A");
          s("forecastWindDirection" + i,"N/A");
          s("forecastWindGust" + i,"N/A");
        }
      }

      var weatherErrorDiv = document.getElementById("weatherErrorDiv");
      var weatherError = document.getElementById("weatherError");
      if (data.weatherError && data.weatherError !== "") {
        weatherError.innerHTML = "Error: " + data.weatherError;
        weatherErrorDiv.style.display = "block";
      } else {
        weatherErrorDiv.style.display = "none";
      }

      drawSkyplane();
      drawPositions(azimuth, elevation, setpoint_az, setpoint_el, kalmanAz, kalmanEl);
      drawWindDirection(data.currentWindDirection, data.weatherDataValid === "YES");
    }
  };
  xhr.ontimeout = function() { pollInFlight = false; };
  xhr.onerror = function() { pollInFlight = false; };
  xhr.send();
}, 250);

function toggleWindSafety(){toggle("windSafetyToggle","/windSafetyOn","/windSafetyOff");setTimeout(fetchConfig,500)}
function toggleWindBasedHome(){toggle("windBasedHomeToggle","/windBasedHomeOn","/windBasedHomeOff");setTimeout(fetchConfig,500)}
function toggleDirectionLock(){var c=document.getElementById("disableDirectionLockToggle").checked;xget(c?"/directionLockOff":"/directionLockOn")}
function toggleExtendedEl(){toggle("extendedElToggle","/extendedElOn","/extendedElOff")}
function toggleAutoHome(){toggle("autoHomeToggle","/autoHomeOn","/autoHomeOff");setTimeout(fetchConfig,500)}
function toggleSmoothTracking(){toggle("smoothTrackingToggle","/smoothTrackingOn","/smoothTrackingOff");setTimeout(fetchConfig,500)}
function toggleWeatherPolling(){toggle("weatherPolling","/weatherOn","/weatherOff");setTimeout(fetchConfig,500)}

function nudgeSetpoint(axis, delta) {
  var currentSpan = document.getElementById("setpoint_" + axis);
  var current = parseFloat(currentSpan.innerHTML);
  if (isNaN(current)) return;
  var newValue = current + delta;

  if (axis === "az") {
    newValue = ((newValue % 360) + 360) % 360;
  } else {
    var extendedEl = document.getElementById("extendedElToggle");
    var minEl = (extendedEl && extendedEl.checked) ? -90 : 0;
    if (newValue < minEl) newValue = minEl;
    if (newValue > 90) newValue = 90;
  }

  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/update_variable", true);
  xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  xhr.send("new_setpoint_" + axis + "=" + newValue.toFixed(2));
}

function updateLogDisplay() {
  var errorTextArea = document.getElementById("errorMessages");
  if (logScrollPaused) return;
  if (logLines.length > 0) {
    errorTextArea.value = logLines.join('\n');
    errorTextArea.className = "has-errors";
    errorTextArea.scrollTop = errorTextArea.scrollHeight;
  } else {
    errorTextArea.value = "";
    errorTextArea.className = "";
  }
  updatePauseButtonText();
}

function toggleLogScrollPause() {
  logScrollPaused = !logScrollPaused;
  if (!logScrollPaused) updateLogDisplay();
  updatePauseButtonText();
}

function updatePauseButtonText() {
  var pauseBtn = document.getElementById('pauseScrollBtn');
  if (pauseBtn) {
    pauseBtn.innerHTML = logScrollPaused ? 'Resume' : 'Pause';
    pauseBtn.style.backgroundColor = logScrollPaused ? '#28a745' : '#ffc107';
  }
}

function clearErrorMessages() {
  logLines = [];
  updateLogDisplay();
}

function updateDebugLevelDropdown(currentLevel) {
  var dropdown = document.getElementById("debugLevel");
  if (dropdown) {
    dropdown.value = currentLevel;
    var levelNames = ["NONE", "ERROR", "WARN", "INFO", "DEBUG", "VERBOSE"];
    var currentLevelDisplay = document.getElementById("currentDebugLevel");
    if (currentLevelDisplay && currentLevel >= 0 && currentLevel <= 5) {
      currentLevelDisplay.innerHTML = currentLevel + " (" + levelNames[currentLevel] + ")";
    }
  }
}

function toggleSerialOutput() {
  var switchState = document.getElementById("disableSerialOutput").checked;
  xget("/setSerialOutputDisabled?disabled=" + switchState);
}

var canvas = document.getElementById("skyplaneCanvas");
var ctx = canvas.getContext("2d");

var centerX = canvas.width / 2;
var centerY = canvas.height / 2;
var radius = Math.min(centerX, centerY) - 20;

function drawSkyplane() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.textAlign = "center";
  ctx.font = "16px Arial";

  for (let i = 4; i >= 0; i--) {
    const currentRadius = radius * (i / 4);
    ctx.beginPath();
    ctx.arc(centerX, centerY, currentRadius, 0, 2 * Math.PI);
    ctx.strokeStyle = i === 4 ? 'black' : 'gray';
    ctx.stroke();
  }

  ctx.beginPath();
  for (let az = 0; az < 360; az += 45) {
    const rad = az * (Math.PI / 180);
    const x = centerX + radius * Math.cos(rad);
    const y = centerY + radius * Math.sin(rad);
    ctx.moveTo(centerX, centerY);
    ctx.lineTo(x, y);
  }
  ctx.strokeStyle = 'gray';
  ctx.stroke();

  ctx.fillStyle = "black";
  const directions = [
    { text: "N", x: centerX, y: centerY - radius - 10 },
    { text: "E", x: centerX + radius + 10, y: centerY + 5 },
    { text: "S", x: centerX, y: centerY + radius + 20 },
    { text: "W", x: centerX - radius - 10, y: centerY + 5 }
  ];
  directions.forEach(dir => {
    ctx.fillText(dir.text, dir.x, dir.y);
  });
}

function drawPositions(azimuth, elevation, setpoint_az, setpoint_el, kalmanAz, kalmanEl) {
  if (elevation >= 350) elevation = 0;

  const toRadians = Math.PI / 180;
  const adjustedAzimuth = -90 * toRadians;

  const positions = [
    { az: azimuth * toRadians + adjustedAzimuth, el: 1 - (elevation / 90), radius: 8, color: 'blue', fill: false },
    { az: setpoint_az * toRadians + adjustedAzimuth, el: 1 - (setpoint_el / 90), radius: 5, color: 'red', fill: true }
  ];

  // Add Kalman estimate as green marker when smooth tracking is active
  if (kalmanAz !== null && kalmanEl !== null && kalmanAz >= 0 && (kalmanEl >= 0 || kalmanEl >= -5)) {
    var kEl = kalmanEl >= 350 ? 0 : Math.max(kalmanEl, 0);
    positions.push({ az: kalmanAz * toRadians + adjustedAzimuth, el: 1 - (kEl / 90), radius: 4, color: '#00cc00', fill: true });
  }

  positions.forEach(pos => {
    let x = centerX + (radius * pos.el) * Math.cos(pos.az);
    let y = centerY + (radius * pos.el) * Math.sin(pos.az);

    const distanceFromCenter = Math.hypot(x - centerX, y - centerY);
    if (distanceFromCenter > radius) {
      const scale = radius / distanceFromCenter;
      x = centerX + (x - centerX) * scale;
      y = centerY + (y - centerY) * scale;
    }

    ctx.beginPath();
    ctx.arc(x, y, pos.radius, 0, 2 * Math.PI);
    if (pos.fill) {
      ctx.fillStyle = pos.color;
      ctx.fill();
    } else {
      ctx.strokeStyle = pos.color;
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  });
}

function drawWindDirection(windDirection, weatherDataValid) {
  if (!weatherDataValid || windDirection === null || windDirection === undefined ||
      windDirection === "N/A" || windDirection === "" || isNaN(parseFloat(windDirection))) {
    return;
  }

  const windDir = parseFloat(windDirection);
  const windRad = (windDir - 90) * (Math.PI / 180);
  const triangleRadius = radius * 1.08;
  const triangleSize = 8;

  const centerTriangleX = centerX + triangleRadius * Math.cos(windRad);
  const centerTriangleY = centerY + triangleRadius * Math.sin(windRad);

  const point1X = centerTriangleX + triangleSize * Math.cos(windRad);
  const point1Y = centerTriangleY + triangleSize * Math.sin(windRad);
  const point2X = centerTriangleX - triangleSize * 0.5 * Math.cos(windRad) + triangleSize * 0.866 * Math.cos(windRad + Math.PI/2);
  const point2Y = centerTriangleY - triangleSize * 0.5 * Math.sin(windRad) + triangleSize * 0.866 * Math.sin(windRad + Math.PI/2);
  const point3X = centerTriangleX - triangleSize * 0.5 * Math.cos(windRad) - triangleSize * 0.866 * Math.cos(windRad + Math.PI/2);
  const point3Y = centerTriangleY - triangleSize * 0.5 * Math.sin(windRad) - triangleSize * 0.866 * Math.sin(windRad + Math.PI/2);

  ctx.beginPath();
  ctx.moveTo(point1X, point1Y);
  ctx.lineTo(point2X, point2Y);
  ctx.lineTo(point3X, point3Y);
  ctx.closePath();
  ctx.fillStyle = '#FF6B35';
  ctx.fill();
  ctx.strokeStyle = '#E55A2B';
  ctx.lineWidth = 1;
  ctx.stroke();
}

drawSkyplane();

function calEl(){xget("/calEl")}
function moveEl(v){xget("/moveEl?value="+v)}
function moveAz(v){xget("/moveAz?value="+v)}

function toggleCal(){toggle("toggleCal","/calon","/caloff")}
function setSingleMotorMode(){toggle("singleMotorMode","/setSingleMotorModeOn","/setSingleMotorModeOff")}
function stellariumOn(){toggle("stellariumOn","/stellariumOn","/stellariumOff");setTimeout(fetchConfig,500)}

function updateVariable() {
  var azValue = parseFloat(document.getElementById("new_setpoint_az").value);
  var elValue = parseFloat(document.getElementById("new_setpoint_el").value);
  var errorMessageAz = document.getElementById("error_message_az");
  var errorMessageEl = document.getElementById("error_message_el");
  var valid = true;

  if (isNaN(azValue) || azValue < -360 || azValue > 360) {
    errorMessageAz.textContent = "Please enter a valid Azimuth between -360 and 360.";
    valid = false;
  } else {
    errorMessageAz.textContent = "";
  }

  var extendedEl = document.getElementById("extendedElToggle");
  var minEl = (extendedEl && extendedEl.checked) ? -90 : 0;
  if (isNaN(elValue) || elValue < minEl || elValue > 90) {
    errorMessageEl.textContent = "Please enter a valid Elevation between " + minEl + " and 90.";
    valid = false;
  } else {
    errorMessageEl.textContent = "";
  }

  if (!valid) return false;

  document.getElementById("new_setpoint_az").value = azValue.toFixed(2);
  document.getElementById("new_setpoint_el").value = elValue.toFixed(2);
  return true;
}

function submitHome() {
  fetch('/submitHome', { method: 'POST' });
}

function stopMotors() {
  fetch('/stopMotors', { method: 'POST' });
}

function toggleHotspotMode() {
  var hotspotCheckbox = document.getElementById("hotspot");
  var ssidField = document.getElementById("ssid");
  var passwordField = document.getElementById("password");

  if (hotspotCheckbox.checked) {
    ssidField.value = "discoverydish_HOTSPOT";
    passwordField.value = "discoverydish";
    ssidField.disabled = true;
    passwordField.disabled = true;
  } else {
    ssidField.value = "";
    passwordField.value = "";
    ssidField.disabled = false;
    passwordField.disabled = false;
  }
}

function confirmRestartESP32(){confirmPost("Are you sure you want to restart the ESP32?\n\nThis will temporarily disconnect the web interface and interrupt any ongoing operations.","/restart")}
function confirmResetNeedsUnwind(){confirmPost("Are you sure you want to reset Needs Unwind flag?\n\nThis could cause the rotator to over rotate and tangle cables.","/resetNeedsUnwind")}
function confirmResetEEPROM(){confirmPost("WARNING: Are you sure you want to reset the EEPROM?\n\nThis will erase all saved settings and return the device to factory defaults. This action cannot be undone.","/resetEEPROM")}

function setDebugLevel() {
  var debugLevel = document.getElementById("debugLevel").value;
  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/setDebugLevel", true);
  xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  xhr.send("debugLevel=" + debugLevel);
  return false;
}

function toggleAdvancedParams() {
  var checkbox = document.getElementById("showAdvanced");
  var section = document.getElementById("advancedParamsSection");
  section.style.display = checkbox.checked ? "block" : "none";
}

function forceWeatherUpdate() {
  xget("/forceWeatherUpdate", function(x) {
    if (x.status == 200) {
      alert("Weather update requested. Data will refresh shortly.");
    } else {
      var errorMsg = "Failed to force weather update";
      if (x.responseText) {
        errorMsg += ": " + x.responseText;
      }
      if (x.responseText.includes("Location not configured")) {
        errorMsg += "\nPlease set your location first.";
      } else if (x.responseText.includes("API key not configured")) {
        errorMsg += "\nPlease set your WeatherAPI.com API key first.";
      }
      alert(errorMsg);
    }
  });
}

function getMyLocation() {
  if (navigator.geolocation) {
    navigator.geolocation.getCurrentPosition(
      function(position) {
        var lat = position.coords.latitude;
        var lon = position.coords.longitude;
        document.getElementById('latitude').value = lat.toFixed(6);
        document.getElementById('longitude').value = lon.toFixed(6);
        alert("Location detected: " + lat.toFixed(6) + ", " + lon.toFixed(6) +
              "\nClick 'Set Location' to save these coordinates.");
      },
      function(error) {
        var errorMsg = "";
        switch(error.code) {
          case error.PERMISSION_DENIED: errorMsg = "Location access denied by user."; break;
          case error.POSITION_UNAVAILABLE: errorMsg = "Location information is unavailable."; break;
          case error.TIMEOUT: errorMsg = "Location request timed out."; break;
          default: errorMsg = "An unknown error occurred."; break;
        }
        alert("Error getting location: " + errorMsg);
      },
      { enableHighAccuracy: true, timeout: 10000, maximumAge: 300000 }
    );
  } else {
    alert("Geolocation is not supported by this browser.");
  }
}

function formatWeatherTime(timeStr) {
  if (!timeStr || timeStr === "N/A" || timeStr === "") return "N/A";
  try {
    var spaceIndex = timeStr.indexOf(' ');
    if (spaceIndex === -1) return timeStr;
    var timePart = timeStr.substring(spaceIndex + 1);
    if (timePart.length >= 5 && timePart.indexOf(':') !== -1) return timePart;
    return timeStr;
  } catch (e) {
    return timeStr;
  }
}

function formatWindDirection(degrees) {
  if (!degrees || degrees === "N/A" || isNaN(degrees)) return "N/A";
  var dir = parseFloat(degrees);
  var directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                   "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"];
  var index = Math.round(dir / 22.5) % 16;
  return dir.toFixed(0) + "° (" + directions[index] + ")";
}

