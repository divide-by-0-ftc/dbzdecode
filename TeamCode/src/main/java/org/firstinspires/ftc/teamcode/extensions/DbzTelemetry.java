package org.firstinspires.ftc.teamcode.extensions;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by nitrocalcite on 2/12/18.
 */

public class DbzTelemetry implements Telemetry {
    private Telemetry telemetry;
    private List<Data> accumulatedEntries = new ArrayList<>();

    public DbzTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void addAccumulatedEntry(String caption, Object object){
        accumulatedEntries.add(new Data(caption, object));
        sendAccumulatedEntries();
    }

    public void addAccumulatedEntry(String caption){
        addAccumulatedEntry(caption, null);
    }

    public void clearAccumulatedEntries(){
        accumulatedEntries = new ArrayList<>();
        sendAccumulatedEntries();
    }

    public void sendAccumulatedEntries(){
        for (Data data : accumulatedEntries){
            if (data.object != null)
                telemetry.addData(data.caption, data.object);
            else
                telemetry.addLine(data.caption);
        }
        telemetry.update();
    }

    public boolean update() {
        return telemetry.update();
    }

    public Log log() {
        return telemetry.log();
    }

    public boolean isAutoClear() {
        return telemetry.isAutoClear();
    }

    public void setAutoClear(boolean autoClear) {
        telemetry.setAutoClear(autoClear);
    }

    public int getMsTransmissionInterval() {
        return telemetry.getMsTransmissionInterval();
    }

    public void setMsTransmissionInterval(int msTransmissionInterval) {
        telemetry.setMsTransmissionInterval(msTransmissionInterval);
    }

    public String getItemSeparator() {
        return telemetry.getItemSeparator();
    }

    public void setItemSeparator(String itemSeparator) {
        telemetry.setItemSeparator(itemSeparator);
    }

    public String getCaptionValueSeparator() {
        return telemetry.getCaptionValueSeparator();
    }

    public void setCaptionValueSeparator(String captionValueSeparator) {
        telemetry.setCaptionValueSeparator(captionValueSeparator);
    }

    public void setDisplayFormat(DisplayFormat displayFormat) {

    }


    public Object addAction(Runnable action) {
        return telemetry.addAction(action);
    }

    public boolean removeAction(Object token) {
        return telemetry.removeAction(token);
    }

    @Override
    public void speak(String text) {

    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {

    }

    public Item addData(String caption, String format, Object... args) {
        return telemetry.addData(caption, format, args);
    }

    public Item addData(String caption, Object value) {
        return telemetry.addData(caption, value);
    }

    public <T> Item addData(String caption, Func<T> valueProducer) {
        return telemetry.addData(caption, valueProducer);
    }

    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return telemetry.addData(caption, format, valueProducer);
    }

    public Line addLine() {
        return telemetry.addLine();
    }

    public Line addLine(String lineCaption) {
        return telemetry.addLine(lineCaption);
    }

    public boolean removeItem(Item item) {
        return telemetry.removeItem(item);
    }

    public boolean removeLine(Line line) {
        return telemetry.removeLine(line);
    }

    public void clear() {
        telemetry.clear();
    }

    public void clearAll() {
        telemetry.clearAll();
    }

    class Data {
        private String caption;
        private Object object;

        Data(String caption, Object object){
            this.caption = caption;
            this.object = object;
        }
    }
}
