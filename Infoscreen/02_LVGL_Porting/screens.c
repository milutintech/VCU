#include <string.h>

#include "screens.h"
#include "images.h"
#include "fonts.h"
#include "actions.h"
#include "vars.h"
#include "styles.h"
#include "ui.h"

#include <string.h>

objects_t objects;
lv_obj_t *tick_value_change_obj;

void create_screen_main() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.main = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 480);
    {
        lv_obj_t *parent_obj = obj;
        {
            // mainGaugeSoc
            lv_obj_t *obj = lv_meter_create(parent_obj);
            objects.main_gauge_soc = obj;
            lv_obj_set_pos(obj, 90, 48);
            lv_obj_set_size(obj, 300, 300);
            {
                lv_meter_scale_t *scale = lv_meter_add_scale(obj);
                lv_meter_set_scale_ticks(obj, scale, 41, 1, 5, lv_color_hex(0xffa8c685));
                lv_meter_set_scale_major_ticks(obj, scale, 8, 3, 10, lv_color_hex(0xffa8c685), 10);
                lv_meter_set_scale_range(obj, scale, 0, 100, 300, 120);
                {
                    lv_meter_indicator_t *indicator = lv_meter_add_needle_line(obj, scale, 5, lv_color_hex(0xff151719), -28);
                    lv_meter_set_indicator_value(obj, indicator, 40);
                }
            }
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff151719), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_16, LV_PART_TICKS | LV_STATE_DEFAULT);
        }
        {
            // mainBtnSettings
            lv_obj_t *obj = lv_imgbtn_create(parent_obj);
            objects.main_btn_settings = obj;
            lv_obj_set_pos(obj, 25, 391);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, 64);
            lv_imgbtn_set_src(obj, LV_IMGBTN_STATE_RELEASED, NULL, &img_settings, NULL);
            lv_imgbtn_set_src(obj, LV_IMGBTN_STATE_PRESSED, NULL, &img_settings, NULL);
            lv_obj_add_event_cb(obj, action_set_global_eez_event, LV_EVENT_PRESSED, (void *)0);
        }
        {
            // mainLableVolt
            lv_obj_t *obj = lv_label_create(parent_obj);
            objects.main_lable_volt = obj;
            lv_obj_set_pos(obj, 176, 397);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "400V");
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // mainBarPwr
            lv_obj_t *obj = lv_bar_create(parent_obj);
            objects.main_bar_pwr = obj;
            lv_obj_set_pos(obj, 30, 91);
            lv_obj_set_size(obj, 28, 215);
            lv_bar_set_range(obj, 0, 450);
            lv_bar_set_value(obj, 25, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff5f6f50), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 80, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xffa8c686), LV_PART_INDICATOR | LV_STATE_DEFAULT);
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            lv_obj_set_pos(obj, 19, 48);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Strom");
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            lv_obj_set_pos(obj, 413, 48);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "TEMP");
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            // mainBarTemp
            lv_obj_t *obj = lv_bar_create(parent_obj);
            objects.main_bar_temp = obj;
            lv_obj_set_pos(obj, 422, 100);
            lv_obj_set_size(obj, 28, 215);
            lv_bar_set_range(obj, 0, 110);
            lv_bar_set_value(obj, 25, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xff5f6f50), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_opa(obj, 80, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xffa8c686), LV_PART_INDICATOR | LV_STATE_DEFAULT);
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            lv_obj_set_pos(obj, 415, 66);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "110°C");
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            lv_obj_set_pos(obj, 23, 66);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "450A");
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}

void tick_screen_main() {
}

void create_screen_settings() {
    lv_obj_t *obj = lv_obj_create(0);
    objects.settings = obj;
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 480, 480);
    {
        lv_obj_t *parent_obj = obj;
        {
            lv_obj_t *obj = lv_imgbtn_create(parent_obj);
            objects.obj0 = obj;
            lv_obj_set_pos(obj, 25, 391);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, 64);
            lv_imgbtn_set_src(obj, LV_IMGBTN_STATE_RELEASED, NULL, &img_back, NULL);
            lv_imgbtn_set_src(obj, LV_IMGBTN_STATE_PRESSED, NULL, &img_back, NULL);
            lv_obj_add_event_cb(obj, action_set_global_eez_event, LV_EVENT_PRESSED, (void *)0);
        }
        {
            lv_obj_t *obj = lv_slider_create(parent_obj);
            lv_obj_set_pos(obj, 25, 58);
            lv_obj_set_size(obj, 430, 23);
            lv_slider_set_value(obj, 25, LV_ANIM_OFF);
        }
        {
            lv_obj_t *obj = lv_slider_create(parent_obj);
            lv_obj_set_pos(obj, 25, 154);
            lv_obj_set_size(obj, 430, 23);
            lv_slider_set_value(obj, 25, LV_ANIM_OFF);
        }
        {
            lv_obj_t *obj = lv_slider_create(parent_obj);
            lv_obj_set_pos(obj, 25, 273);
            lv_obj_set_size(obj, 430, 23);
            lv_slider_set_value(obj, 25, LV_ANIM_OFF);
        }
        {
            lv_obj_t *obj = lv_label_create(parent_obj);
            lv_obj_set_pos(obj, 176, 25);
            lv_obj_set_size(obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_label_set_text(obj, "Max DC Current");
            lv_obj_set_style_text_font(obj, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}

void tick_screen_settings() {
}


void create_screens() {
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    
    create_screen_main();
    create_screen_settings();
}

typedef void (*tick_screen_func_t)();

tick_screen_func_t tick_screen_funcs[] = {
    tick_screen_main,
    tick_screen_settings,
};

void tick_screen(int screen_index) {
    tick_screen_funcs[screen_index]();
}
