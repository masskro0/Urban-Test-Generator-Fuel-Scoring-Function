$scriptpath = $MyInvocation.MyCommand.Path
$dir = Split-Path $scriptpath
Write-host "Project path: $dir"
python -m venv venv
.\venv\Scripts\activate
pip install PyScaffold
pip install Shapely-1.6.4.post2-cp37-cp37m-win_amd64.whl
pip install -r requirements.txt
python .\setup.py
Write-host "Press [Y] to install packages for the traffic light detection system, else [N]."
$key = $Host.UI.RawUI.ReadKey()
if ($key.Character -eq 'Y') {
  pip install -r .\evaluation\traffic_lights\detector\requirements.txt
}
Read-Host -Prompt "Setup completed. Press Enter to exit"