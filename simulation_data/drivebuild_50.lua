local M = {}
local sh = require('ge/extensions/scenario/scenariohelper')

local function onRaceStart()
    local modeFile = io.open('ego_movementMode', 'w')
    modeFile:write('_BEAMNG')
    modeFile:close()
    sh.setAiPath({vehicleName='ego', waypoints={'wp_ego_0', 'wp_ego_1', 'wp_ego_2', 'wp_ego_3', 'wp_ego_4', 'wp_ego_5', 'wp_ego_6', 'wp_ego_7', 'wp_ego_8', 'wp_ego_9', 'wp_ego_10', 'wp_ego_11', 'wp_ego_12', 'wp_ego_13', 'wp_ego_14', 'wp_ego_15', 'wp_ego_16', 'wp_ego_17', 'wp_ego_18', 'wp_ego_19', 'wp_ego_20', 'wp_ego_21', 'wp_ego_22', 'wp_ego_23', 'wp_ego_24', 'wp_ego_25', 'wp_ego_26', 'wp_ego_27', 'wp_ego_28', 'wp_ego_29', 'wp_ego_30', 'wp_ego_31', 'wp_ego_32', 'wp_ego_33', 'wp_ego_34', 'wp_ego_35', 'wp_ego_36', 'wp_ego_37', 'wp_ego_38', 'wp_ego_39', 'wp_ego_40', 'wp_ego_41', 'wp_ego_42', 'wp_ego_43', 'wp_ego_44', 'wp_ego_45', 'wp_ego_46', 'wp_ego_47', 'wp_ego_48', 'wp_ego_49', 'wp_ego_50', 'wp_ego_51', 'wp_ego_52', 'wp_ego_53', 'wp_ego_54', 'wp_ego_55', 'wp_ego_56', 'wp_ego_57', 'wp_ego_58', 'wp_ego_59', 'wp_ego_60', 'wp_ego_61', 'wp_ego_62', 'wp_ego_63', 'wp_ego_64', 'wp_ego_65', 'wp_ego_66', 'wp_ego_67', 'wp_ego_68', 'wp_ego_69', 'wp_ego_70', 'wp_ego_71', 'wp_ego_72', 'wp_ego_73', 'wp_ego_74', 'wp_ego_75', 'wp_ego_76', 'wp_ego_77', 'wp_ego_78', 'wp_ego_79', 'wp_ego_80', 'wp_ego_81', 'wp_ego_82', 'wp_ego_83', 'wp_ego_84', 'wp_ego_85', 'wp_ego_86', 'wp_ego_87', 'wp_ego_88', 'wp_ego_89', 'wp_ego_90', 'wp_ego_91', 'wp_ego_92', 'wp_ego_93', 'wp_ego_94', 'wp_ego_95', 'wp_ego_96', 'wp_ego_97', 'wp_ego_98', 'wp_ego_99', 'wp_ego_100', 'wp_ego_101', 'wp_ego_102', 'wp_ego_103', 'wp_ego_104', 'wp_ego_105', 'wp_ego_106', 'wp_ego_107'}, driveInLane='on', routeSpeed=13.88888888888889, routeSpeedMode='set'})
end

M.onRaceStart = onRaceStart
return M