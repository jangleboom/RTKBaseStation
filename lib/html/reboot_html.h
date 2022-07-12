#ifndef REBOOT_HTML_H
#define REBOOT_HTML_H


const char REBOOT_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>

<head>
    <meta content="text/html" ; charset="UTF-8" ; http-equiv="content-type">
    <meta name="viewport" content="width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
    <title>RTK base setup</title>
    <style>
        body {
            background-color: #4180C8;
            font-family: Lato, Helvetica, Roboto, sans-serif;
            color: GhostWhite;
            text-align: center;
            border: 1em;
        }

        .center {
            margin-left: auto;
            margin-right: auto;
        }

        progress {
            position: relative;
            accent-color: #F0A03C;
        }
    </style>

    <script type="text/javascript">
        let duration = 5
        var timeleft = duration;
        var downloadTimer = setInterval(function() {
            
            if (timeleft <= -1) {
                clearInterval(downloadTimer);
            }

            document.getElementById("progressBar").value = duration - timeleft;
            let label = `in ${timeleft} second(s)`;
            document.getElementById("countdown").innerHTML = label;
            if ((timeleft) == -1) {
                document.getElementById("countdown").innerHTML = "done";
            }
            timeleft -= 1;
        }, 1000);
    </script>
</head>

<body>
    <h2>RTK Base Station</h2>
    <h3>restart device</h3>

    <span id="countdown">in 5 second(s)</span> <br>
    <progress value="0" max="5" id="progressBar"> </progress><br>
    <p>
    <table class=center>
        <tr>
            <td style="text-align:right;"> SSID: </td>
            <td style="text-align:left;"> %next_ssid% </td>
        </tr>
        <tr>
            <td style="text-align:right;"> ADDR: </td>
            <td style="text-align:left;"> <a href="http://%next_addr%">http://%next_addr%</a> </td>
        </tr>
    </table>
</p>

</body>

</html>

)rawliteral";


#endif /* HTML_H */


