local M = {}
local sh = require('ge/extensions/scenario/scenariohelper')

local function onRaceStart()
    local modeFile = io.open('ego_movementMode', 'w')
    modeFile:write('_BEAMNG')
    modeFile:close()
    sh.setAiPath({vehicleName='ego', waypoints={'wp_ego_0', 'wp_ego_1', 'wp_ego_2', 'wp_ego_3', 'wp_ego_4', 'wp_ego_5', 'wp_ego_6', 'wp_ego_7', 'wp_ego_8', 'wp_ego_9', 'wp_ego_10', 'wp_ego_11', 'wp_ego_12', 'wp_ego_13', 'wp_ego_14', 'wp_ego_15', 'wp_ego_16', 'wp_ego_17', 'wp_ego_18', 'wp_ego_19', 'wp_ego_20', 'wp_ego_21', 'wp_ego_22', 'wp_ego_23', 'wp_ego_24', 'wp_ego_25', 'wp_ego_26', 'wp_ego_27', 'wp_ego_28', 'wp_ego_29', 'wp_ego_30', 'wp_ego_31', 'wp_ego_32', 'wp_ego_33', 'wp_ego_34', 'wp_ego_35', 'wp_ego_36', 'wp_ego_37', 'wp_ego_38', 'wp_ego_39', 'wp_ego_40', 'wp_ego_41', 'wp_ego_42', 'wp_ego_43', 'wp_ego_44', 'wp_ego_45', 'wp_ego_46', 'wp_ego_47'}, driveInLane='on', routeSpeed=13.88888888888889, routeSpeedMode='set'})
end

M.onRaceStart = onRaceStart
return M