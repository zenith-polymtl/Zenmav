from core import Zenmav

drone = Zenmav('COM12', GCS = True)

drone.download_all_params('oui_baguette')
